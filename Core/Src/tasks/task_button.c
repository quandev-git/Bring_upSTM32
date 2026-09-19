#include "door_control.h"

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

