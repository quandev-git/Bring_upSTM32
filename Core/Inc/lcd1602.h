#ifndef LCD1602_H
#define LCD1602_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define LCD1602_ADDRESS_0X27  (0x27U << 1)
#define LCD1602_ADDRESS_0X3F  (0x3FU << 1)

HAL_StatusTypeDef LCD1602_Init(I2C_HandleTypeDef *hi2c, uint16_t address);
HAL_StatusTypeDef LCD1602_Clear(void);
HAL_StatusTypeDef LCD1602_SetCursor(uint8_t row, uint8_t column);
HAL_StatusTypeDef LCD1602_Print(const char *text);
HAL_StatusTypeDef LCD1602_Backlight(uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif /* LCD1602_H */
