#include "door_control.h"
#include "pi_uart.h"

/* Pi sends one ASCII line per detection: CAR_DETECTED\n (115200 8N1). */
void vTaskUART_Pi(void *pv)
{
    (void)pv;
    char line[20];
    uint8_t length = 0;
    bool discard_until_newline = false;
    uint8_t byte;
    PiUart_StartRx();

    for (;;) {
        if (PiUart_ReadByte(&byte, portMAX_DELAY) != pdTRUE) continue;
        if (byte == '\r') continue;
        if (byte == '\n') {
            if (!discard_until_newline) {
                line[length] = '\0';
                if (strcmp(line, "CAR_DETECTED") == 0) {
                    yolo_result_t event = {.match = true};
                    (void)xQueueSend(xYoloResultQueue, &event, 0);
                }
            }
            length = 0;
            discard_until_newline = false;
        } else if (!discard_until_newline && byte >= 32U && byte <= 126U &&
                   length < sizeof(line) - 1U) {
            line[length++] = (char)byte;
        } else {
            discard_until_newline = true;
        }
    }
}
