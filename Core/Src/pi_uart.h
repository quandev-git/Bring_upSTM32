#ifndef PI_UART_H
#define PI_UART_H

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"

HAL_StatusTypeDef PiUart_Init(void);
void PiUart_StartRx(void);
BaseType_t PiUart_ReadByte(uint8_t *byte, TickType_t wait_ticks);

#endif
