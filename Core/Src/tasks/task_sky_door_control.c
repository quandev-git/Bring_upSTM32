#include "door_control.h"

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

