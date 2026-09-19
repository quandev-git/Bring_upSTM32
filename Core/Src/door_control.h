#ifndef DOOR_CONTROL_H
#define DOOR_CONTROL_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "rc522.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ---------------------------------------------------------------------- */
/*  Config constants — adjust to your wiring                              */
/* ---------------------------------------------------------------------- */
#define CURRENT_LIMIT_MA          3000U   /* obstruction cutoff threshold */
#define CURRENT_SAMPLE_PERIOD_MS  20U
#define PWM_RAMP_STEP             5U      /* duty % per ramp tick */
#define PWM_RAMP_PERIOD_MS        20U
#define PWM_MAX_DUTY              100U
#define DOOR_TRAVEL_TIMEOUT_MS    8000U   /* safety: max time to open/close */

/* Digital IR obstacle sensor on PB8. Set the active level for your module. */
#define IR_OBSTACLE_SAMPLE_MS      20U
#define IR_OBSTACLE_ACTIVE_STATE   GPIO_PIN_RESET

#define MAX_PARK_SLOTS            3U

/* If 1: when the lot is full, manual button override is ALSO blocked —
   no way to open the front door until a car exits. If 0 (default): full
   only blocks automatic NFC/YOLO entry; the button still wins as an
   emergency/manual override, consistent with the original arbitration
   rule ("button always wins"). Confirm which behavior you actually want
   before flipping this. */
#define PARKING_FULL_BLOCKS_OVERRIDE 0

/* ---------------------------------------------------------------------- */
/*  Shared enums                                                          */
/* ---------------------------------------------------------------------- */
typedef enum {
    DOOR_IDLE_CLOSED = 0,
    DOOR_OPENING,
    DOOR_HOLDING_OPEN,
    DOOR_CLOSING,
    DOOR_IDLE_OPEN,
    DOOR_OBSTRUCTED,
    DOOR_MANUAL_OVERRIDE,
    DOOR_FAULT
} door_state_t;

typedef enum {
    CMD_NONE = 0,
    CMD_OPEN,
    CMD_CLOSE,
    CMD_STOP,
    CMD_MANUAL_OVERRIDE
} door_cmd_t;

typedef enum {
    DOOR_FRONT = 0,
    DOOR_SKY,
    DOOR_COUNT
} door_id_t;

/* ---------------------------------------------------------------------- */
/*  Messages passed between tasks                                        */
/* ---------------------------------------------------------------------- */
typedef struct {
    char line1[21];
    char line2[21];
} lcd_update_t;

typedef struct {
    door_id_t   door;
    door_cmd_t  cmd;
} door_cmd_msg_t;

typedef struct {
    bool     match;          /* true = plate/visual matched known car */
    char     plate[16];
    uint8_t  confidence;     /* 0-100 */
    uint32_t request_id;     /* correlates with the request STM32 sent */
} yolo_result_t;

typedef struct {
    door_id_t door;
    uint16_t  current_ma;
} obstruction_evt_t;

/* Direction of a confirmed car crossing, reported by Pi4 once a vehicle
   actually crosses the gate line — NOT the same event as EVT_CAR_MATCH
   (which only says "this car is authorized", fired on approach). The
   counter must react to crossing, not to authorization, or a car that
   is granted access but then reverses/aborts would be miscounted. */
typedef enum {
    DIR_ENTRY = 0,
    DIR_EXIT
} crossing_dir_t;

typedef struct {
    crossing_dir_t direction;
} parking_crossing_evt_t;

/* Digital IR sensor reports an obstacle, not a distance. */
typedef struct {
    bool detected;
} obstacle_evt_t;

typedef enum {
    BUZZ_NONE = 0,
    BUZZ_SLOW,            /* car approaching, still safe distance */
    BUZZ_FAST,             /* getting close */
    BUZZ_CONTINUOUS,       /* critical — about to hit something */
    BUZZ_ACCESS_MISMATCH,  /* NFC/YOLO disagree, distinct short pattern */
    BUZZ_DENIED             /* e.g. lot full — single long beep */
} buzzer_pattern_t;

/* Reuse the short pulse pattern for confirmed entry. */
#define BUZZ_ENTRY_CONFIRM BUZZ_SLOW

/* Legacy two-step verification state used by the optional access-control
   task. The active one-slot flow is owned by door_control.c. */
typedef enum {
    VERIFY_IDLE = 0,
    VERIFY_CAM_PENDING,   /* camera matched, waiting for NFC -> entry attempt */
    VERIFY_NFC_PENDING    /* NFC swiped, waiting for camera -> exit attempt */
} verify_state_t;

#define TWO_STEP_TIMEOUT_MS   15000U   /* max time allowed between step 1 and step 2 */

/* One-shot NFC swipe event — pushed by vTaskNFCRead on each new valid
   swipe (edge-triggered), not a continuous "card present" flag. */
typedef struct {
    bool valid;
} nfc_evt_t;

/* ---------------------------------------------------------------------- */
/*  Handle for one motor channel (BTS7960)                                */
/* ---------------------------------------------------------------------- */
typedef struct {
    door_id_t          id;
    door_state_t       state;
    TIM_HandleTypeDef *pwm_timer;
    uint32_t           pwm_channel_fwd;
    uint32_t           pwm_channel_rev;
    GPIO_TypeDef       *en_port;
    uint16_t            en_pin;
    SemaphoreHandle_t   mutex;      /* owns the H-bridge */
    uint8_t              current_duty;
    TickType_t            move_start_tick;
} motor_channel_t;

/* ---------------------------------------------------------------------- */
/*  Global RTOS objects (defined in door_control.c)                       */
/* ---------------------------------------------------------------------- */
extern QueueHandle_t xLcdQueue;
extern QueueHandle_t xFrontDoorCmdQueue;
extern QueueHandle_t xSkyDoorCmdQueue;
extern QueueHandle_t xYoloResultQueue;
extern QueueHandle_t xObstructionQueue;
extern QueueHandle_t xParkingCrossingQueue;
extern QueueHandle_t xObstacleQueue;
extern QueueHandle_t xBuzzerQueue;
extern QueueHandle_t xNfcEventQueue;
extern SemaphoreHandle_t xParkingSlotSem;   /* counting semaphore, 0..MAX_PARK_SLOTS free slots */

extern motor_channel_t frontMotor;
extern motor_channel_t skyMotor;

/* ---------------------------------------------------------------------- */
/*  Public API                                                            */
/* ---------------------------------------------------------------------- */
void DoorControl_Init(void);

/* Active one-slot application. The legacy DoorControl_Init() above remains
   available for the older, larger task set. */
HAL_StatusTypeDef DoorControl_Start(I2C_HandleTypeDef *hi2c);
void DoorControl_Tick(uint32_t now_ms);
void DoorControl_OnIr(bool active);
void DoorControl_OnCard(const rc522_uid_t *uid, uint32_t now_ms);
void DoorControl_OnCarDetected(uint32_t now_ms);

typedef enum {
    DOOR_STARTING = 0,
    DOOR_BOARD_ERROR,
    DOOR_LCD_NOT_FOUND,
    DOOR_LCD_ERROR,
    DOOR_RC522_ERROR,
    DOOR_RTOS_ERROR,
    DOOR_PWM_ERROR,
    DOOR_READY,
    DOOR_CARD_DETECTED,
    DOOR_UART_ERROR
} door_control_status_t;

extern volatile door_control_status_t door_control_status;
extern volatile uint8_t door_lcd_address7;
extern volatile uint8_t door_occupied;
extern volatile uint32_t door_pi_detect_count;

/* Arbitration */
door_cmd_t sky_arbitrate(bool button_evt, bool rain_detected);

/* Parking slot pool — backed by a counting semaphore */
bool    ParkingSlots_Available(void);   /* true if at least 1 free slot */
uint8_t ParkingSlots_Count(void);       /* current free slot count, 0..MAX_PARK_SLOTS */

/* State machines — called once per tick from their owning task */
void front_door_execute(motor_channel_t *m, door_cmd_t cmd);
void sky_door_execute(motor_channel_t *m, door_cmd_t cmd);

/* Task entry points */
void vTaskButton(void *pv);
void vTaskCurrentMonitor(void *pv);
void vTaskFrontDoorControl(void *pv);
void vTaskSkyDoorControl(void *pv);
void vTaskNFCRead(void *pv);
void vTaskUART_Pi(void *pv);
void vTaskAccessControl(void *pv);
void vTaskRainSensor(void *pv);
void vTaskSkyDoorLogic(void *pv);
void vTaskParkingCounter(void *pv);
void vTaskIRObstacle(void *pv);
void vTaskLCD(void *pv);
void vTaskBuzzer(void *pv);
void vTaskWatchdog(void *pv);

#endif /* DOOR_CONTROL_H */
