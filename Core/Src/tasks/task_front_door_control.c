#include "door_control.h"

/* ---------------------------------------------------------------------- */
/*  vTaskFrontDoorControl / vTaskSkyDoorControl                           */
/*  Owns the motor, drains its command queue, drives the state machine.   */
/* ---------------------------------------------------------------------- */
void vTaskFrontDoorControl(void *pv)
{
    (void)pv;
    door_cmd_msg_t msg;
    door_cmd_t     latest_cmd = CMD_NONE;
    TickType_t open_finished_at = 0;
    bool auto_close_pending = false;

    for (;;) {
        if (xQueueReceive(xFrontDoorCmdQueue, &msg, pdMS_TO_TICKS(PWM_RAMP_PERIOD_MS)) == pdTRUE) {
            if (msg.door == DOOR_FRONT) latest_cmd = msg.cmd;
        }
        bool ir_active = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) ==
                         IR_OBSTACLE_ACTIVE_STATE;
        if (frontMotor.state == DOOR_CLOSING && ir_active) {
            latest_cmd = CMD_STOP;
            lcd_update_t display = {0};
            strcpy(display.line1, "LOI: IR CAN CUA");
            strcpy(display.line2, "RESET DE TIEP TUC");
            (void)xQueueSend(xLcdQueue, &display, 0);
        } else if (frontMotor.state == DOOR_IDLE_OPEN && auto_close_pending &&
                   !ir_active &&
                   (xTaskGetTickCount() - open_finished_at) >=
                       pdMS_TO_TICKS(3000U)) {
            latest_cmd = CMD_CLOSE;
            auto_close_pending = false;
        }
        door_state_t previous_state = frontMotor.state;
        front_door_execute(&frontMotor, latest_cmd);
        if (frontMotor.state == DOOR_IDLE_OPEN &&
            previous_state != DOOR_IDLE_OPEN) {
            open_finished_at = xTaskGetTickCount();
            auto_close_pending = true;
        }
        latest_cmd = CMD_NONE; /* command consumed each tick; re-sent while held */
    }
}
