#include "door_control.h"

/* Active-high buzzer module on PA11. */
void vTaskBuzzer(void *pv)
{
    (void)pv;
    buzzer_pattern_t pattern;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    for (;;) {
        if (xQueueReceive(xBuzzerQueue, &pattern, portMAX_DELAY) == pdTRUE &&
            pattern == BUZZ_ENTRY_CONFIRM) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(200));
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        }
    }
}
