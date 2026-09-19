#include "lcd2004.h"

/* Common PCF8574 backpack: P0=RS, P1=RW, P2=E, P3=backlight, P4..P7=D4..D7. */
#define LCD_RS         0x01U
#define LCD_ENABLE     0x04U
#define LCD_BACKLIGHT  0x08U

static I2C_HandleTypeDef *lcd_i2c;
static uint16_t lcd_address;
static uint8_t lcd_light = LCD_BACKLIGHT;

static HAL_StatusTypeDef lcd_write_raw(uint8_t data)
{
    return HAL_I2C_Master_Transmit(lcd_i2c, lcd_address, &data, 1, 50);
}

static HAL_StatusTypeDef lcd_write_nibble(uint8_t data)
{
    uint8_t base = data | lcd_light;
    if (lcd_write_raw(base) != HAL_OK ||
        lcd_write_raw(base | LCD_ENABLE) != HAL_OK ||
        lcd_write_raw(base) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

static HAL_StatusTypeDef lcd_write_byte(uint8_t data, uint8_t mode)
{
    if (lcd_write_nibble((data & 0xF0U) | mode) != HAL_OK ||
        lcd_write_nibble(((data << 4) & 0xF0U) | mode) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef LCD2004_Init(I2C_HandleTypeDef *hi2c, uint16_t address)
{
    if (hi2c == NULL) return HAL_ERROR;
    lcd_i2c = hi2c;
    lcd_address = address;
    lcd_light = LCD_BACKLIGHT;

    HAL_Delay(50);
    if (lcd_write_nibble(0x30U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(5);
    if (lcd_write_nibble(0x30U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);
    if (lcd_write_nibble(0x30U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);
    if (lcd_write_nibble(0x20U) != HAL_OK) return HAL_ERROR;
    if (lcd_write_byte(0x28U, 0) != HAL_OK ||
        lcd_write_byte(0x08U, 0) != HAL_OK ||
        LCD2004_Clear() != HAL_OK ||
        lcd_write_byte(0x06U, 0) != HAL_OK ||
        lcd_write_byte(0x0CU, 0) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef LCD2004_Clear(void)
{
    if (lcd_i2c == NULL || lcd_write_byte(0x01U, 0) != HAL_OK) return HAL_ERROR;
    HAL_Delay(2);
    return HAL_OK;
}

HAL_StatusTypeDef LCD2004_SetCursor(uint8_t row, uint8_t column)
{
    static const uint8_t offsets[4] = {0x00U, 0x40U, 0x14U, 0x54U};
    if (lcd_i2c == NULL || row >= 4U || column >= 20U) return HAL_ERROR;
    return lcd_write_byte(0x80U | (offsets[row] + column), 0);
}

HAL_StatusTypeDef LCD2004_Print(const char *text)
{
    if (lcd_i2c == NULL || text == NULL) return HAL_ERROR;
    while (*text != '\0') {
        if (lcd_write_byte((uint8_t)*text++, LCD_RS) != HAL_OK) return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef LCD2004_Backlight(uint8_t enabled)
{
    if (lcd_i2c == NULL) return HAL_ERROR;
    lcd_light = enabled ? LCD_BACKLIGHT : 0;
    return lcd_write_raw(lcd_light);
}
