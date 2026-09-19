#include "door_control.h"

/* Samples RFID and IR. DoorControl owns the entry/exit state and count. */
void vTaskNFCRead(void *pv)
{
    (void)pv;
    bool stable_ir = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) ==
                     IR_OBSTACLE_ACTIVE_STATE;
    bool candidate_ir = stable_ir;
    uint8_t candidate_samples = 0;
    bool card_latched = false;
    uint8_t absent_samples = 0;
    DoorControl_OnIr(stable_ir);

    for (;;) {
        uint32_t now_ms = xTaskGetTickCount();
        DoorControl_Tick(now_ms);

        yolo_result_t pi_event;
        while (xQueueReceive(xYoloResultQueue, &pi_event, 0) == pdTRUE) {
            if (pi_event.match) DoorControl_OnCarDetected(now_ms);
        }

        bool raw_ir = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) ==
                      IR_OBSTACLE_ACTIVE_STATE;
        if (raw_ir != candidate_ir) {
            candidate_ir = raw_ir;
            candidate_samples = 1U;
        } else if (candidate_samples < 3U) {
            ++candidate_samples;
        }
        if (candidate_samples >= 3U && stable_ir != candidate_ir) {
            stable_ir = candidate_ir;
            DoorControl_OnIr(stable_ir);
        }

        rc522_uid_t uid;
        if (RC522_ReadUid(&uid)) {
            absent_samples = 0;
            if (!card_latched) {
                card_latched = true;
                DoorControl_OnCard(&uid, xTaskGetTickCount());
            }
        } else if (++absent_samples >= 3U) {
            card_latched = false;
            absent_samples = 3U;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
