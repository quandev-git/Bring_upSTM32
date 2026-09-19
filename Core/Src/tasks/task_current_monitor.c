#include "door_control.h"

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

