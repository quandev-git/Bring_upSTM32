#include "door_control.h"

void vTaskRainSensor(void *pv)
{
    (void)pv;
    for (;;) {
        /* TODO: read rain sensor GPIO/ADC, update shared flag for
           vTaskSkyDoorLogic */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

