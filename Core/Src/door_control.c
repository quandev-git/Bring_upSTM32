#include "door_control.h"

#include "lcd2004.h"
#include "pi_uart.h"

/* ---------------------------------------------------------------------- */
/*  Global RTOS objects                                                   */
/* ---------------------------------------------------------------------- */
QueueHandle_t xLcdQueue          = NULL;
QueueHandle_t xFrontDoorCmdQueue = NULL;
QueueHandle_t xSkyDoorCmdQueue   = NULL;
QueueHandle_t xYoloResultQueue   = NULL;
QueueHandle_t xObstructionQueue  = NULL;
QueueHandle_t xParkingCrossingQueue = NULL;
QueueHandle_t xObstacleQueue = NULL;
QueueHandle_t xBuzzerQueue = NULL;
QueueHandle_t xNfcEventQueue = NULL;
SemaphoreHandle_t xParkingSlotSem   = NULL;

motor_channel_t frontMotor;
motor_channel_t skyMotor;

/* ---------------------------------------------------------------------- */
/*  Init — call from main() before vTaskStartScheduler()                  */
/* ---------------------------------------------------------------------- */
void DoorControl_Init(void)
{
    xLcdQueue          = xQueueCreate(8,  sizeof(lcd_update_t));
    xFrontDoorCmdQueue = xQueueCreate(4,  sizeof(door_cmd_msg_t));
    xSkyDoorCmdQueue   = xQueueCreate(4,  sizeof(door_cmd_msg_t));
    xYoloResultQueue   = xQueueCreate(4,  sizeof(yolo_result_t));
    xObstructionQueue  = xQueueCreate(4,  sizeof(obstruction_evt_t));
    xParkingCrossingQueue = xQueueCreate(4, sizeof(parking_crossing_evt_t));
    xObstacleQueue = xQueueCreate(4, sizeof(obstacle_evt_t));
    xBuzzerQueue = xQueueCreate(4, sizeof(buzzer_pattern_t));
    xNfcEventQueue = xQueueCreate(4, sizeof(nfc_evt_t));

    /* Counting semaphore = the pool of free parking slots.
       Max count = MAX_PARK_SLOTS, starts full (all slots free). */
    xParkingSlotSem = xSemaphoreCreateCounting(MAX_PARK_SLOTS, MAX_PARK_SLOTS);

    frontMotor.id            = DOOR_FRONT;
    frontMotor.state         = DOOR_IDLE_CLOSED;
    frontMotor.current_duty  = 0;
    frontMotor.mutex         = xSemaphoreCreateMutex();
    /* TODO: assign frontMotor.pwm_timer / channels / en_port / en_pin
       to match your CubeMX TIM + GPIO configuration */

    skyMotor.id            = DOOR_SKY;
    skyMotor.state         = DOOR_IDLE_CLOSED;
    skyMotor.current_duty  = 0;
    skyMotor.mutex         = xSemaphoreCreateMutex();
    /* TODO: assign skyMotor.pwm_timer / channels / en_port / en_pin */

    configASSERT(xLcdQueue && xFrontDoorCmdQueue && xSkyDoorCmdQueue &&
                 xYoloResultQueue && xObstructionQueue &&
                 xParkingCrossingQueue && xParkingSlotSem &&
                 xObstacleQueue && xBuzzerQueue && xNfcEventQueue &&
                 frontMotor.mutex && skyMotor.mutex);
}

/* ---------------------------------------------------------------------- */
/*  Parking slot pool helpers                                             */
/* ---------------------------------------------------------------------- */
bool ParkingSlots_Available(void)
{
    return uxSemaphoreGetCount(xParkingSlotSem) > 0;
}

uint8_t ParkingSlots_Count(void)
{
    return (uint8_t)uxSemaphoreGetCount(xParkingSlotSem);
}

/* ---------------------------------------------------------------------- */
/*  Arbitration                                                           */
/* ---------------------------------------------------------------------- */
door_cmd_t sky_arbitrate(bool button_evt, bool rain_detected)
{
    if (button_evt)     return CMD_MANUAL_OVERRIDE;
    return rain_detected ? CMD_CLOSE : CMD_OPEN;
}

/* ---------------------------------------------------------------------- */
/*  Low-level motor helpers                                               */
/* ---------------------------------------------------------------------- */
static void motor_set_duty_forward(motor_channel_t *m, uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(m->pwm_timer, m->pwm_channel_fwd,
                           (m->pwm_timer->Init.Period * duty) / 100U);
    __HAL_TIM_SET_COMPARE(m->pwm_timer, m->pwm_channel_rev, 0);
    m->current_duty = duty;
}

static void motor_set_duty_reverse(motor_channel_t *m, uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(m->pwm_timer, m->pwm_channel_rev,
                           (m->pwm_timer->Init.Period * duty) / 100U);
    __HAL_TIM_SET_COMPARE(m->pwm_timer, m->pwm_channel_fwd, 0);
    m->current_duty = duty;
}

static void motor_stop(motor_channel_t *m)
{
    __HAL_TIM_SET_COMPARE(m->pwm_timer, m->pwm_channel_fwd, 0);
    __HAL_TIM_SET_COMPARE(m->pwm_timer, m->pwm_channel_rev, 0);
    m->current_duty = 0;
}

/* Ramp helper: call repeatedly (every PWM_RAMP_PERIOD_MS) until it
   returns true (target duty reached). Non-blocking ramp. */
static bool motor_ramp_to(motor_channel_t *m, uint8_t target_duty, bool forward)
{
    if (m->current_duty < target_duty) {
        uint8_t next = m->current_duty + PWM_RAMP_STEP;
        if (next > target_duty) next = target_duty;
        forward ? motor_set_duty_forward(m, next) : motor_set_duty_reverse(m, next);
    } else if (m->current_duty > target_duty) {
        uint8_t next = m->current_duty - PWM_RAMP_STEP;
        if (next < target_duty) next = target_duty;
        forward ? motor_set_duty_forward(m, next) : motor_set_duty_reverse(m, next);
    }
    return (m->current_duty == target_duty);
}

/* ---------------------------------------------------------------------- */
/*  Front door state machine                                              */
/*  Called once per control tick with the latest arbitrated command.      */
/* ---------------------------------------------------------------------- */
void front_door_execute(motor_channel_t *m, door_cmd_t cmd)
{
    if (xSemaphoreTake(m->mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return; /* another task owns the H-bridge this tick, try again next */
    }

    switch (m->state) {

    case DOOR_IDLE_CLOSED:
        if (cmd == CMD_OPEN || cmd == CMD_MANUAL_OVERRIDE) {
            m->move_start_tick = xTaskGetTickCount();
            m->state = DOOR_OPENING;
        }
        break;

    case DOOR_OPENING:
        if (motor_ramp_to(m, PWM_MAX_DUTY, true)) {
            /* full duty reached — in a real system you'd wait for a limit
               switch / encoder here instead of a pure timeout */
        }
        if (cmd == CMD_STOP) {
            motor_stop(m);
            m->state = DOOR_IDLE_CLOSED; /* or DOOR_FAULT if mid-travel */
        } else if ((xTaskGetTickCount() - m->move_start_tick) >
                   pdMS_TO_TICKS(DOOR_TRAVEL_TIMEOUT_MS)) {
            motor_stop(m);
            m->state = DOOR_HOLDING_OPEN; /* assume reached open end-stop */
        }
        break;

    case DOOR_HOLDING_OPEN:
        motor_stop(m);
        m->state = DOOR_IDLE_OPEN;
        break;

    case DOOR_IDLE_OPEN:
        if (cmd == CMD_CLOSE) {
            m->move_start_tick = xTaskGetTickCount();
            m->state = DOOR_CLOSING;
        }
        break;

    case DOOR_CLOSING:
        if (motor_ramp_to(m, PWM_MAX_DUTY, false)) {
            /* ramped to full reverse duty */
        }
        if (cmd == CMD_STOP) {
            motor_stop(m);
            m->state = DOOR_FAULT;
        } else if ((xTaskGetTickCount() - m->move_start_tick) >
                   pdMS_TO_TICKS(DOOR_TRAVEL_TIMEOUT_MS)) {
            motor_stop(m);
            m->state = DOOR_IDLE_CLOSED;
        }
        break;

    case DOOR_OBSTRUCTED:
        /* vTaskCurrentMonitor pushed us here — reverse briefly, then idle */
        motor_set_duty_reverse(m, 30);
        vTaskDelay(pdMS_TO_TICKS(300));
        motor_stop(m);
        m->state = DOOR_IDLE_OPEN;
        break;

    case DOOR_MANUAL_OVERRIDE:
        /* handled by button task directly; fall back to idle once released */
        m->state = DOOR_IDLE_CLOSED;
        break;

    case DOOR_FAULT:
    default:
        motor_stop(m);
        /* stays in FAULT until an explicit reset command/path is added */
        break;
    }

    xSemaphoreGive(m->mutex);
}

/* ---------------------------------------------------------------------- */
/*  Sky door state machine — same shape, kept separate for independent    */
/*  tuning (different travel time, no "manual override wins forever"      */
/*  nuance needed here since rain logic is periodic, not event-driven)    */
/* ---------------------------------------------------------------------- */
void sky_door_execute(motor_channel_t *m, door_cmd_t cmd)
{
    if (xSemaphoreTake(m->mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    switch (m->state) {
    case DOOR_IDLE_CLOSED:
        if (cmd == CMD_OPEN) {
            m->move_start_tick = xTaskGetTickCount();
            m->state = DOOR_OPENING;
        }
        break;

    case DOOR_OPENING:
        motor_ramp_to(m, PWM_MAX_DUTY, true);
        if ((xTaskGetTickCount() - m->move_start_tick) >
            pdMS_TO_TICKS(DOOR_TRAVEL_TIMEOUT_MS)) {
            motor_stop(m);
            m->state = DOOR_IDLE_OPEN;
        }
        break;

    case DOOR_IDLE_OPEN:
        if (cmd == CMD_CLOSE) {
            m->move_start_tick = xTaskGetTickCount();
            m->state = DOOR_CLOSING;
        }
        break;

    case DOOR_CLOSING:
        motor_ramp_to(m, PWM_MAX_DUTY, false);
        if ((xTaskGetTickCount() - m->move_start_tick) >
            pdMS_TO_TICKS(DOOR_TRAVEL_TIMEOUT_MS)) {
            motor_stop(m);
            m->state = DOOR_IDLE_CLOSED;
        }
        break;

    case DOOR_OBSTRUCTED:
        motor_stop(m);
        m->state = DOOR_IDLE_OPEN;
        break;

    default:
        motor_stop(m);
        break;
    }

    xSemaphoreGive(m->mutex);
}

/* One-slot application flow. The motor state machines above are retained. */
#define DOOR_CONFIRM_WINDOW_MS 15000U

typedef enum {
    FLOW_WAIT_ENTRY = 0,
    FLOW_WAIT_ENTRY_IR,
    FLOW_WAIT_IR_CLEAR,
    FLOW_WAIT_EXIT_CARD
} flow_phase_t;

typedef enum {
    FLOW_NO_ACTION = 0,
    FLOW_OPEN_ENTRY,
    FLOW_COUNT_FULL,
    FLOW_READY_EXIT,
    FLOW_EXIT,
    FLOW_REJECT_SAME_CARD,
    FLOW_REJECT_IR_BLOCKED,
    FLOW_WAIT_CAMERA,
    FLOW_WAIT_CARD,
    FLOW_CONFIRM_TIMEOUT
} flow_action_t;

typedef struct {
    flow_phase_t phase;
    uint8_t occupied;
    bool ir_active;
    rc522_uid_t entry_uid;
    rc522_uid_t pending_uid;
    bool pending_card;
    bool pending_camera;
    uint32_t pending_since;
} door_flow_t;

static door_flow_t active_flow;
static SPI_HandleTypeDef active_spi2;
static TIM_HandleTypeDef active_tim2;
volatile door_control_status_t door_control_status = DOOR_STARTING;
volatile uint8_t door_lcd_address7 = 0;
volatile uint8_t door_occupied = 0;
volatile uint32_t door_pi_detect_count = 0;

static void flow_clear_pending(void)
{
    active_flow.pending_card = false;
    active_flow.pending_camera = false;
    memset(&active_flow.pending_uid, 0, sizeof(active_flow.pending_uid));
}

static flow_action_t flow_approve_entry(void)
{
    active_flow.entry_uid = active_flow.pending_uid;
    active_flow.phase = FLOW_WAIT_ENTRY_IR;
    flow_clear_pending();
    return FLOW_OPEN_ENTRY;
}

static flow_action_t flow_ir_changed(bool active)
{
    if (active_flow.ir_active == active) return FLOW_NO_ACTION;
    active_flow.ir_active = active;
    if (active_flow.phase == FLOW_WAIT_ENTRY_IR && active) {
        active_flow.occupied = 1U;
        active_flow.phase = FLOW_WAIT_IR_CLEAR;
        return FLOW_COUNT_FULL;
    }
    if (active_flow.phase == FLOW_WAIT_IR_CLEAR && !active) {
        active_flow.phase = FLOW_WAIT_EXIT_CARD;
        return FLOW_READY_EXIT;
    }
    return FLOW_NO_ACTION;
}

static flow_action_t flow_card_seen(const rc522_uid_t *uid, uint32_t now_ms)
{
    if (uid == NULL || (uid->length != 4U && uid->length != 7U &&
                        uid->length != 10U)) return FLOW_NO_ACTION;
    if (active_flow.phase != FLOW_WAIT_ENTRY &&
        active_flow.phase != FLOW_WAIT_EXIT_CARD) return FLOW_NO_ACTION;
    if (active_flow.ir_active) return FLOW_REJECT_IR_BLOCKED;

    if (active_flow.phase == FLOW_WAIT_EXIT_CARD) {
        if (uid->length == active_flow.entry_uid.length &&
            memcmp(uid->bytes, active_flow.entry_uid.bytes, uid->length) == 0)
            return FLOW_REJECT_SAME_CARD;
        active_flow.occupied = 0U;
        active_flow.phase = FLOW_WAIT_ENTRY;
        memset(&active_flow.entry_uid, 0, sizeof(active_flow.entry_uid));
        flow_clear_pending();
        return FLOW_EXIT;
    }

    if (!active_flow.pending_card && !active_flow.pending_camera)
        active_flow.pending_since = now_ms;
    active_flow.pending_uid = *uid;
    active_flow.pending_card = true;
    if (active_flow.pending_camera) return flow_approve_entry();
    return FLOW_WAIT_CAMERA;
}

static flow_action_t flow_car_seen(uint32_t now_ms)
{
    if (active_flow.phase != FLOW_WAIT_ENTRY || active_flow.ir_active)
        return FLOW_NO_ACTION;
    if (!active_flow.pending_card && !active_flow.pending_camera)
        active_flow.pending_since = now_ms;
    active_flow.pending_camera = true;
    if (active_flow.pending_card) return flow_approve_entry();
    return FLOW_WAIT_CARD;
}

static flow_action_t flow_check_timeout(uint32_t now_ms)
{
    if ((active_flow.pending_card || active_flow.pending_camera) &&
        (uint32_t)(now_ms - active_flow.pending_since) >
            DOOR_CONFIRM_WINDOW_MS) {
        flow_clear_pending();
        return FLOW_CONFIRM_TIMEOUT;
    }
    return FLOW_NO_ACTION;
}

static void door_queue_lcd(const char *line1, const char *line2)
{
    lcd_update_t message = {0};
    strncpy(message.line1, line1, sizeof(message.line1) - 1U);
    strncpy(message.line2, line2, sizeof(message.line2) - 1U);
    (void)xQueueSend(xLcdQueue, &message, 0);
}

static void door_request_open(void)
{
    door_cmd_msg_t command = {.door = DOOR_FRONT, .cmd = CMD_OPEN};
    (void)xQueueSend(xFrontDoorCmdQueue, &command, pdMS_TO_TICKS(50));
}

static void door_handle_action(flow_action_t action)
{
    switch (action) {
    case FLOW_OPEN_ENTRY:
        door_request_open();
        door_queue_lcd("XE VAO 0/1", "CUA DANG MO");
        break;
    case FLOW_COUNT_FULL: {
        buzzer_pattern_t beep = BUZZ_ENTRY_CONFIRM;
        (void)xQueueSend(xBuzzerQueue, &beep, 0);
        door_queue_lcd("XE: 1/1 - DA DAY", "CHO XE QUA IR");
        break;
    }
    case FLOW_READY_EXIT:
        door_queue_lcd("XE: 1/1 - DA DAY", "QUET THE KHAC DE RA");
        break;
    case FLOW_EXIT:
        door_request_open();
        door_queue_lcd("XE RA - XE: 0/1", "CON 1 SLOT");
        break;
    case FLOW_REJECT_SAME_CARD:
        door_queue_lcd("XE: 1/1 - DA DAY", "PHAI DUNG THE KHAC");
        break;
    case FLOW_REJECT_IR_BLOCKED:
        door_queue_lcd("IR DANG CO VAT", "CHUA THE MO CUA");
        break;
    case FLOW_WAIT_CAMERA:
        door_queue_lcd("DA QUET THE", "CHO PI XAC NHAN XE");
        break;
    case FLOW_WAIT_CARD:
        door_queue_lcd("PI DA THAY XE", "QUET THE RFID");
        break;
    case FLOW_CONFIRM_TIMEOUT:
        door_queue_lcd("XE: 0/1 - CON SLOT", "QUET LAI / PI DETECT");
        break;
    case FLOW_NO_ACTION:
    default:
        break;
    }
}

void DoorControl_Tick(uint32_t now_ms)
{
    door_handle_action(flow_check_timeout(now_ms));
}

void DoorControl_OnIr(bool active)
{
    door_handle_action(flow_ir_changed(active));
    door_occupied = active_flow.occupied;
}

void DoorControl_OnCard(const rc522_uid_t *uid, uint32_t now_ms)
{
    door_control_status = DOOR_CARD_DETECTED;
    if (frontMotor.state == DOOR_FAULT) {
        door_queue_lcd("LOI CUA", "RESET DE TIEP TUC");
    } else if (frontMotor.state != DOOR_IDLE_CLOSED) {
        door_queue_lcd("CUA DANG CHAY", "CHO CUA DONG");
    } else {
        door_handle_action(flow_card_seen(uid, now_ms));
        door_occupied = active_flow.occupied;
    }
}

void DoorControl_OnCarDetected(uint32_t now_ms)
{
    ++door_pi_detect_count;
    if (frontMotor.state == DOOR_IDLE_CLOSED)
        door_handle_action(flow_car_seen(now_ms));
}

static uint16_t door_find_lcd_address(I2C_HandleTypeDef *hi2c)
{
    static const uint8_t preferred[] = {0x27U, 0x3FU};
    for (size_t i = 0; i < sizeof(preferred); ++i) {
        uint16_t address = (uint16_t)preferred[i] << 1;
        if (HAL_I2C_IsDeviceReady(hi2c, address, 2, 100) == HAL_OK)
            return address;
    }
    for (uint8_t address7 = 0x20U; address7 <= 0x3FU; ++address7) {
        if ((address7 > 0x27U && address7 < 0x38U) ||
            address7 == preferred[0] || address7 == preferred[1]) continue;
        uint16_t address = (uint16_t)address7 << 1;
        if (HAL_I2C_IsDeviceReady(hi2c, address, 2, 100) == HAL_OK)
            return address;
    }
    return 0;
}

static void door_show_error(const char *line2)
{
    (void)LCD2004_Clear();
    (void)LCD2004_SetCursor(0, 0);
    (void)LCD2004_Print("LOI KHOI DONG");
    (void)LCD2004_SetCursor(1, 0);
    (void)LCD2004_Print(line2);
}

static HAL_StatusTypeDef door_board_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef pwm = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Motor bridge outputs stay disabled until PWM has started at zero. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9 | GPIO_PIN_12, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_8;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    active_spi2.Instance = SPI2;
    active_spi2.Init.Mode = SPI_MODE_MASTER;
    active_spi2.Init.Direction = SPI_DIRECTION_2LINES;
    active_spi2.Init.DataSize = SPI_DATASIZE_8BIT;
    active_spi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    active_spi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    active_spi2.Init.NSS = SPI_NSS_SOFT;
    active_spi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    active_spi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    active_spi2.Init.TIMode = SPI_TIMODE_DISABLE;
    active_spi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    active_spi2.Init.CRCPolynomial = 7;
    if (HAL_SPI_Init(&active_spi2) != HAL_OK) return HAL_ERROR;

    active_tim2.Instance = TIM2;
    active_tim2.Init.Prescaler = 0;
    active_tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    active_tim2.Init.Period = 399; /* 20 kHz at the current 8 MHz timer clock. */
    active_tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    active_tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&active_tim2) != HAL_OK) return HAL_ERROR;

    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.Pulse = 0;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&active_tim2, &pwm, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&active_tim2, &pwm, TIM_CHANNEL_2) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef DoorControl_Start(I2C_HandleTypeDef *hi2c)
{
    door_control_status = DOOR_STARTING;
    door_lcd_address7 = 0;
    door_occupied = 0;
    door_pi_detect_count = 0;
    memset(&active_flow, 0, sizeof(active_flow));

    if (hi2c == NULL || door_board_init() != HAL_OK) {
        door_control_status = DOOR_BOARD_ERROR;
        return HAL_ERROR;
    }
    uint16_t lcd_address = door_find_lcd_address(hi2c);
    if (lcd_address == 0U) {
        door_control_status = DOOR_LCD_NOT_FOUND;
        return HAL_ERROR;
    }
    door_lcd_address7 = (uint8_t)(lcd_address >> 1);
    if (LCD2004_Init(hi2c, lcd_address) != HAL_OK) {
        door_control_status = DOOR_LCD_ERROR;
        return HAL_ERROR;
    }
    (void)LCD2004_SetCursor(0, 0);
    (void)LCD2004_Print("DANG KHOI DONG");
    if (RC522_Init(&active_spi2) != HAL_OK) {
        door_control_status = DOOR_RC522_ERROR;
        door_show_error("KIEM TRA RC522 SPI2");
        return HAL_ERROR;
    }

    xFrontDoorCmdQueue = xQueueCreate(4, sizeof(door_cmd_msg_t));
    xLcdQueue = xQueueCreate(4, sizeof(lcd_update_t));
    xBuzzerQueue = xQueueCreate(2, sizeof(buzzer_pattern_t));
    xYoloResultQueue = xQueueCreate(4, sizeof(yolo_result_t));
    frontMotor.mutex = xSemaphoreCreateMutex();
    if (!xFrontDoorCmdQueue || !xLcdQueue || !xBuzzerQueue ||
        !xYoloResultQueue || !frontMotor.mutex) {
        door_control_status = DOOR_RTOS_ERROR;
        door_show_error("THIEU BO NHO RTOS");
        return HAL_ERROR;
    }
    if (PiUart_Init() != HAL_OK) {
        door_control_status = DOOR_UART_ERROR;
        door_show_error("KIEM TRA USART1");
        return HAL_ERROR;
    }

    frontMotor.id = DOOR_FRONT;
    frontMotor.state = DOOR_IDLE_CLOSED;
    frontMotor.pwm_timer = &active_tim2;
    frontMotor.pwm_channel_fwd = TIM_CHANNEL_1;
    frontMotor.pwm_channel_rev = TIM_CHANNEL_2;
    frontMotor.en_port = GPIOA;
    frontMotor.en_pin = GPIO_PIN_2 | GPIO_PIN_3;
    frontMotor.current_duty = 0;

    if (xTaskCreate(vTaskNFCRead, "EntryFlow", 384, NULL, 2, NULL) != pdPASS ||
        xTaskCreate(vTaskFrontDoorControl, "FrontMotor", 256, NULL, 3, NULL) != pdPASS ||
        xTaskCreate(vTaskLCD, "LCD2004", 256, NULL, 1, NULL) != pdPASS ||
        xTaskCreate(vTaskBuzzer, "Buzzer", 128, NULL, 1, NULL) != pdPASS ||
        xTaskCreate(vTaskUART_Pi, "PiUART", 192, NULL, 2, NULL) != pdPASS) {
        door_control_status = DOOR_RTOS_ERROR;
        door_show_error("KHONG TAO DUOC TASK");
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&active_tim2, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&active_tim2, TIM_CHANNEL_2) != HAL_OK) {
        door_control_status = DOOR_PWM_ERROR;
        door_show_error("KIEM TRA TIM2 PWM");
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
    door_control_status = DOOR_READY;
    return HAL_OK;
}
