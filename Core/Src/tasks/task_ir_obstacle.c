#include "door_control.h"

void vTaskIRObstacle(void *pv)
{
    (void)pv;
    TickType_t lastWake = xTaskGetTickCount();
    bool last_detected = false;
    bool first_sample = true;

    for (;;) {
        bool detected = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == IR_OBSTACLE_ACTIVE_STATE;
        if (first_sample || detected != last_detected) {
            obstacle_evt_t evt = { .detected = detected };
            buzzer_pattern_t pattern = detected ? BUZZ_CONTINUOUS : BUZZ_NONE;
            xQueueSend(xObstacleQueue, &evt, 0);
            xQueueSend(xBuzzerQueue, &pattern, 0);
            last_detected = detected;
            first_sample = false;
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IR_OBSTACLE_SAMPLE_MS));
    }
}