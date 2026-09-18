#include "door_control.h"

/* ---------------------------------------------------------------------- */
/*  Global RTOS objects                                                   */
/* ---------------------------------------------------------------------- */
QueueHandle_t xLcdQueue          = NULL;
QueueHandle_t xFrontDoorCmdQueue = NULL;
QueueHandle_t xSkyDoorCmdQueue   = NULL;
QueueHandle_t xYoloResultQueue   = NULL;
QueueHandle_t xObstructionQueue  = NULL;
QueueHandle_t xParkingCrossingQueue = NULL;
QueueHandle_t xProximityQueue = NULL;
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
    xProximityQueue = xQueueCreate(4, sizeof(proximity_evt_t));
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
                 xProximityQueue && xBuzzerQueue && xNfcEventQueue &&
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

/* ---------------------------------------------------------------------- */
/*  vTaskCurrentMonitor — highest-priority safety task                    */
/*  Polls both BTS7960 IS pins via ADC/DMA, posts an obstruction event    */
/*  if either channel exceeds the current limit while moving.             */
/* ---------------------------------------------------------------------- */
void vTaskCurrentMonitor(void *pv)
{
    (void)pv;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        /* TODO: replace with real ADC/DMA read of both IS pins */
        uint16_t front_current_ma = 0; /* = ADC_ReadFrontIS(); */
        uint16_t sky_current_ma   = 0; /* = ADC_ReadSkyIS(); */

        if (front_current_ma > CURRENT_LIMIT_MA &&
            (frontMotor.state == DOOR_OPENING || frontMotor.state == DOOR_CLOSING)) {
            frontMotor.state = DOOR_OBSTRUCTED;
            obstruction_evt_t evt = { .door = DOOR_FRONT, .current_ma = front_current_ma };
            xQueueSend(xObstructionQueue, &evt, 0);
        }

        if (sky_current_ma > CURRENT_LIMIT_MA &&
            (skyMotor.state == DOOR_OPENING || skyMotor.state == DOOR_CLOSING)) {
            skyMotor.state = DOOR_OBSTRUCTED;
            obstruction_evt_t evt = { .door = DOOR_SKY, .current_ma = sky_current_ma };
            xQueueSend(xObstructionQueue, &evt, 0);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CURRENT_SAMPLE_PERIOD_MS));
    }
}

/* ---------------------------------------------------------------------- */
/*  vTaskFrontDoorControl / vTaskSkyDoorControl                           */
/*  Owns the motor, drains its command queue, drives the state machine.   */
/* ---------------------------------------------------------------------- */
void vTaskFrontDoorControl(void *pv)
{
    (void)pv;
    door_cmd_msg_t msg;
    door_cmd_t     latest_cmd = CMD_NONE;

    for (;;) {
        if (xQueueReceive(xFrontDoorCmdQueue, &msg, pdMS_TO_TICKS(PWM_RAMP_PERIOD_MS)) == pdTRUE) {
            if (msg.door == DOOR_FRONT) latest_cmd = msg.cmd;
        }
        front_door_execute(&frontMotor, latest_cmd);
        latest_cmd = CMD_NONE; /* command consumed each tick; re-sent while held */
    }
}

void vTaskSkyDoorControl(void *pv)
{
    (void)pv;
    door_cmd_msg_t msg;
    door_cmd_t     latest_cmd = CMD_NONE;

    for (;;) {
        if (xQueueReceive(xSkyDoorCmdQueue, &msg, pdMS_TO_TICKS(PWM_RAMP_PERIOD_MS)) == pdTRUE) {
            if (msg.door == DOOR_SKY) latest_cmd = msg.cmd;
        }
        sky_door_execute(&skyMotor, latest_cmd);
        latest_cmd = CMD_NONE;
    }
}

/* ---------------------------------------------------------------------- */
/*  vTaskButton — EXTI-driven, debounced manual override for both doors   */
/*  Expects an EXTI ISR to give a binary semaphore per button; poll here  */
/*  for simplicity, swap for xSemaphoreTake(ISR_sem, portMAX_DELAY) if    */
/*  you wire it event-driven instead.                                    */
/* ---------------------------------------------------------------------- */
void vTaskButton(void *pv)
{
    (void)pv;
    for (;;) {
        bool front_btn = false; /* TODO: read debounced GPIO / EXTI flag */
        bool sky_btn   = false; /* TODO: second button or long-press disambiguation */

        if (front_btn) {
            door_cmd_msg_t m = { .door = DOOR_FRONT, .cmd = CMD_MANUAL_OVERRIDE };
            xQueueSend(xFrontDoorCmdQueue, &m, 0);
        }
        if (sky_btn) {
            door_cmd_msg_t m = { .door = DOOR_SKY, .cmd = CMD_MANUAL_OVERRIDE };
            xQueueSend(xSkyDoorCmdQueue, &m, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------------------------------------------------------------------- */
/*  Two-step verification helpers                                         */
/* ---------------------------------------------------------------------- */
static void confirm_entry(void)
{
    if (!ParkingSlots_Available()) {
        /* Both steps were completed correctly, but the lot is full — deny
           at the last moment. This is a real scenario: camera matched,
           driver tapped the card, only then do we know slots ran out
           (e.g. someone else took the last one moments earlier). */
        lcd_update_t full = {0};
        strncpy(full.line1, "PARKING FULL", 16);
        strncpy(full.line2, "Please wait...", 16);
        xQueueSend(xLcdQueue, &full, 0);
        buzzer_pattern_t p = BUZZ_DENIED;
        xQueueSend(xBuzzerQueue, &p, 0);
        return;
    }

    door_cmd_msg_t m = { .door = DOOR_FRONT, .cmd = CMD_OPEN };
    xQueueSend(xFrontDoorCmdQueue, &m, 0);

    /* Direction is known from the verification order itself — no camera
       bbox-size/direction logic needed. Slot accounting (semaphore take)
       happens in vTaskParkingCounter when it drains this event. */
    parking_crossing_evt_t evt = { .direction = DIR_ENTRY };
    xQueueSend(xParkingCrossingQueue, &evt, 0);

    lcd_update_t ok = {0};
    strncpy(ok.line1, "Welcome home", 16);
    xQueueSend(xLcdQueue, &ok, 0);
}

static void confirm_exit(void)
{
    door_cmd_msg_t m = { .door = DOOR_FRONT, .cmd = CMD_OPEN };
    xQueueSend(xFrontDoorCmdQueue, &m, 0);

    parking_crossing_evt_t evt = { .direction = DIR_EXIT };
    xQueueSend(xParkingCrossingQueue, &evt, 0);

    lcd_update_t ok = {0};
    strncpy(ok.line1, "Goodbye!", 16);
    strncpy(ok.line2, "Drive safe", 16);
    xQueueSend(xLcdQueue, &ok, 0);
}

/* ---------------------------------------------------------------------- */
/*  vTaskAccessControl — front-door two-step verification                 */
/*                                                                        */
/*  ENTRY: camera matches the car first, then the driver taps the NFC     */
/*         card within TWO_STEP_TIMEOUT_MS.                               */
/*  EXIT:  the driver taps the card first, then the camera confirms the   */
/*         car within the same window.                                    */
/*  The ORDER the two events arrive in is what tells us the direction —   */
/*  this replaces needing the camera to infer entry/exit from bbox size.  */
/*  Manual button always wins immediately and cancels any pending         */
/*  half-completed sequence.                                              */
/* ---------------------------------------------------------------------- */
void vTaskAccessControl(void *pv)
{
    (void)pv;
    verify_state_t state = VERIFY_IDLE;
    TickType_t     pending_since = 0;
    yolo_result_t  yolo;
    nfc_evt_t      nfc;

    for (;;) {
        bool button_evt = false; /* TODO: shared flag/queue from vTaskButton */
        bool cam_event = (xQueueReceive(xYoloResultQueue, &yolo, 0) == pdTRUE) && yolo.match;
        bool nfc_event = (xQueueReceive(xNfcEventQueue, &nfc, 0) == pdTRUE) && nfc.valid;

        if (button_evt) {
            /* Manual override always wins and clears any stale
               half-verification so it can't leak into the next attempt. */
            state = VERIFY_IDLE;
            door_cmd_msg_t m = { .door = DOOR_FRONT, .cmd = CMD_MANUAL_OVERRIDE };
            xQueueSend(xFrontDoorCmdQueue, &m, 0);
        }

        if (state != VERIFY_IDLE &&
            (xTaskGetTickCount() - pending_since) > pdMS_TO_TICKS(TWO_STEP_TIMEOUT_MS)) {
            lcd_update_t warn = {0};
            strncpy(warn.line1, "VERIFY TIMEOUT", 16);
            strncpy(warn.line2,
                    state == VERIFY_CAM_PENDING ? "No card tapped" : "No cam confirm",
                    16);
            xQueueSend(xLcdQueue, &warn, 0);
            state = VERIFY_IDLE;
        }

        switch (state) {
        case VERIFY_IDLE:
            if (cam_event) {
                state = VERIFY_CAM_PENDING;
                pending_since = xTaskGetTickCount();
                lcd_update_t upd = {0};
                strncpy(upd.line1, "Car recognized", 16);
                strncpy(upd.line2, "Tap card to enter", 16);
                xQueueSend(xLcdQueue, &upd, 0);
            } else if (nfc_event) {
                state = VERIFY_NFC_PENDING;
                pending_since = xTaskGetTickCount();
                lcd_update_t upd = {0};
                strncpy(upd.line1, "Card accepted", 16);
                strncpy(upd.line2, "Confirming exit", 16);
                xQueueSend(xLcdQueue, &upd, 0);
            }
            break;

        case VERIFY_CAM_PENDING:
            if (nfc_event) {
                confirm_entry();
                state = VERIFY_IDLE;
            } else if (cam_event) {
                pending_since = xTaskGetTickCount(); /* car still in frame — refresh window */
            }
            break;

        case VERIFY_NFC_PENDING:
            if (cam_event) {
                confirm_exit();
                state = VERIFY_IDLE;
            } else if (nfc_event) {
                pending_since = xTaskGetTickCount();
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ---------------------------------------------------------------------- */
/*  vTaskSkyDoorLogic — rain-driven arbitration, independent of front     */
/* ---------------------------------------------------------------------- */
void vTaskSkyDoorLogic(void *pv)
{
    (void)pv;
    for (;;) {
        bool button_evt    = false; /* handled separately via vTaskButton if you wire it */
        bool rain_detected = false; /* TODO: shared flag from vTaskRainSensor */

        door_cmd_t cmd = sky_arbitrate(button_evt, rain_detected);
        if (cmd != CMD_NONE) {
            door_cmd_msg_t m = { .door = DOOR_SKY, .cmd = cmd };
            xQueueSend(xSkyDoorCmdQueue, &m, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---------------------------------------------------------------------- */
/*  vTaskLCD — single owner of the display, drains a queue                */
/* ---------------------------------------------------------------------- */
void vTaskLCD(void *pv)
{
    (void)pv;
    lcd_update_t upd;
    for (;;) {
        if (xQueueReceive(xLcdQueue, &upd, portMAX_DELAY) == pdTRUE) {
            /* TODO: replace with your lcd1602/lcd2004 driver calls */
            /* LCD_SetCursor(0,0); LCD_Print(upd.line1);
               LCD_SetCursor(1,0); LCD_Print(upd.line2); */
        }
    }
}

/* ---------------------------------------------------------------------- */
/*  Remaining tasks — skeletons only, fill in per your peripheral drivers */
/* ---------------------------------------------------------------------- */
void vTaskNFCRead(void *pv)
{
    (void)pv;
    for (;;) {
        /* TODO: poll NFC reader. On each NEW valid card presented (edge —
           don't re-fire every poll while the same card sits on the
           reader), validate UID against the whitelist and push exactly
           one event:
             nfc_evt_t evt = { .valid = true };
             xQueueSend(xNfcEventQueue, &evt, 0);
           vTaskAccessControl treats this as one swipe. Debounce/edge
           detection belongs here, not in the FSM. */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskUART_Pi(void *pv)
{
    (void)pv;
    for (;;) {
        /* TODO: receive framed UART message from Pi4, parse into
           yolo_result_t, xQueueSend(xYoloResultQueue, &result, 0); */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void vTaskRainSensor(void *pv)
{
    (void)pv;
    for (;;) {
        /* TODO: read rain sensor GPIO/ADC, update shared flag for
           vTaskSkyDoorLogic */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTaskParkingCounter(void *pv)
{
    (void)pv;
    parking_crossing_evt_t evt;

    for (;;) {
        /* Blocks until Pi4 (via vTaskUART_Pi) reports a confirmed crossing.
           This is deliberately a SEPARATE event from EVT_CAR_MATCH: match
           only means "authorized to enter", fired on approach. Crossing
           means the car actually passed the gate line — that's the only
           thing that should move the counter, so a car that gets access
           but backs out never gets counted. Direction detection itself is
           Pi4-side work (e.g. tracking bounding-box motion across a virtual
           line in the frame, or two beam sensors) — out of scope here. */
        if (xQueueReceive(xParkingCrossingQueue, &evt, portMAX_DELAY) == pdTRUE) {

            if (evt.direction == DIR_ENTRY) {
                if (xSemaphoreTake(xParkingSlotSem, 0) != pdTRUE) {
                    /* Semaphore already at 0 — an entry was confirmed while
                       we had no free slot on record. Should not happen if
                       the full-lot gate above is working; treat as a
                       miscount/anomaly and flag it rather than silently
                       going negative (counting semaphore can't go negative
                       anyway — this branch just means we dropped the
                       event). Log it. */
                    lcd_update_t err = {0};
                    strncpy(err.line1, "COUNT ERROR", 16);
                    strncpy(err.line2, "Entry w/ 0 free", 16);
                    xQueueSend(xLcdQueue, &err, 0);
                }
            } else { /* DIR_EXIT */
                if (xSemaphoreGive(xParkingSlotSem) != pdTRUE) {
                    /* Give failed => already at MAX_PARK_SLOTS. Means an
                       exit was double-reported, or an earlier entry was
                       never counted. Also an anomaly worth surfacing. */
                    lcd_update_t err = {0};
                    strncpy(err.line1, "COUNT ERROR", 16);
                    strncpy(err.line2, "Exit w/ lot full", 16);
                    xQueueSend(xLcdQueue, &err, 0);
                }
            }

            lcd_update_t upd = {0};
            snprintf(upd.line1, sizeof(upd.line1), "Slots: %u/%u",
                      ParkingSlots_Count(), MAX_PARK_SLOTS);
            strncpy(upd.line2, ParkingSlots_Available() ? "" : "LOT FULL", 16);
            xQueueSend(xLcdQueue, &upd, 0);
        }
    }
}

void vTaskParkAssist(void *pv)
{
    (void)pv;
    TickType_t lastWake = xTaskGetTickCount();
    buzzer_pattern_t last_sent = BUZZ_NONE;

    for (;;) {
        /* TODO: replace with real ultrasonic read (HC-SR04 style: trigger
           pulse + echo pulse-width capture via TIM input capture, or a
           ToF/IR module). This task is independent of door motor current
           sensing — it watches the CAR's distance to a wall/object in the
           stall while reversing, not the door mechanism. */
        uint16_t distance_cm = 0xFFFF; /* = UltrasonicRead(); placeholder = "far/no reading" */

        buzzer_pattern_t pattern;
        if (distance_cm > PARK_ASSIST_SAFE_CM) {
            pattern = BUZZ_NONE;
        } else if (distance_cm > PARK_ASSIST_WARN_CM) {
            pattern = BUZZ_NONE; /* within safe zone but not yet warn threshold */
        } else if (distance_cm > PARK_ASSIST_CRITICAL_CM) {
            pattern = BUZZ_SLOW;
        } else {
            pattern = BUZZ_CONTINUOUS;
        }

        /* only post when it changes — avoid spamming the buzzer queue every
           80ms with the same pattern */
        if (pattern != last_sent) {
            xQueueSend(xBuzzerQueue, &pattern, 0);
            last_sent = pattern;
        }

        proximity_evt_t evt = { .distance_cm = distance_cm };
        xQueueSend(xProximityQueue, &evt, 0); /* for logging/LCD if desired */

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PARK_ASSIST_SAMPLE_MS));
    }
}

void vTaskBuzzer(void *pv)
{
    (void)pv;
    buzzer_pattern_t pattern = BUZZ_NONE;

    for (;;) {
        /* Block until a new pattern arrives, but also re-check periodically
           so continuous/slow patterns keep beeping without needing a new
           queue message each cycle. */
        xQueueReceive(xBuzzerQueue, &pattern, pdMS_TO_TICKS(50));

        switch (pattern) {
        case BUZZ_NONE:
            /* TODO: HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); */
            vTaskDelay(pdMS_TO_TICKS(50));
            break;

        case BUZZ_SLOW:
            /* TODO: buzzer ON */
            vTaskDelay(pdMS_TO_TICKS(80));
            /* TODO: buzzer OFF */
            vTaskDelay(pdMS_TO_TICKS(400));
            break;

        case BUZZ_FAST:
            /* TODO: buzzer ON */
            vTaskDelay(pdMS_TO_TICKS(80));
            /* TODO: buzzer OFF */
            vTaskDelay(pdMS_TO_TICKS(120));
            break;

        case BUZZ_CONTINUOUS:
            /* TODO: buzzer ON, held */
            vTaskDelay(pdMS_TO_TICKS(50));
            break;

        case BUZZ_DENIED:
            /* one long beep, then drop back to NONE */
            /* TODO: buzzer ON */
            vTaskDelay(pdMS_TO_TICKS(500));
            /* TODO: buzzer OFF */
            pattern = BUZZ_NONE;
            break;

        case BUZZ_ACCESS_MISMATCH:
            /* short distinct double-beep, then drop back to NONE */
            for (int i = 0; i < 2; i++) {
                /* TODO: buzzer ON */
                vTaskDelay(pdMS_TO_TICKS(100));
                /* TODO: buzzer OFF */
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            pattern = BUZZ_NONE;
            break;

        default:
            pattern = BUZZ_NONE;
            break;
        }
    }
}

void vTaskWatchdog(void *pv)
{
    (void)pv;
    for (;;) {
        /* TODO: check per-task heartbeat timestamps, HAL_IWDG_Refresh()
           only if all tasks are healthy */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
