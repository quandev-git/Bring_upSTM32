#include "door_control.h"

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

