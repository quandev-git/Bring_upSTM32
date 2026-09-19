#include "door_control.h"
#include "lcd2004.h"

static void lcd_print_row(uint8_t row, const char *message)
{
    char padded[21];
    size_t index = 0;
    while (index < 20U && message[index] != '\0') {
        padded[index] = message[index];
        ++index;
    }
    while (index < 20U) padded[index++] = ' ';
    padded[20] = '\0';
    if (LCD2004_SetCursor(row, 0) == HAL_OK) (void)LCD2004_Print(padded);
}

/* ---------------------------------------------------------------------- */
/*  LCD2004 is owned by this task after DoorControl_Start initializes it. */
/* ---------------------------------------------------------------------- */
void vTaskLCD(void *pv)
{
    (void)pv;
    lcd_update_t upd;
    lcd_print_row(0, "XE: 0/1 - CON SLOT");
    lcd_print_row(1, "QUET THE + PI");
    for (;;) {
        if (xQueueReceive(xLcdQueue, &upd, portMAX_DELAY) == pdTRUE) {
            lcd_print_row(0, upd.line1);
            lcd_print_row(1, upd.line2);
        }
    }
}
