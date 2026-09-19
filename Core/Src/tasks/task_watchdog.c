#include "door_control.h"

void vTaskWatchdog(void *pv)
{
    (void)pv;
    for (;;) {
        /* TODO: check per-task heartbeat timestamps, HAL_IWDG_Refresh()
           only if all tasks are healthy */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
