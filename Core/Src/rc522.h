#ifndef RC522_H
#define RC522_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

typedef struct {
    uint8_t length;
    uint8_t bytes[10];
} rc522_uid_t;

HAL_StatusTypeDef RC522_Init(SPI_HandleTypeDef *hspi);
bool RC522_CardPresent(void);
bool RC522_ReadUid(rc522_uid_t *uid);

#endif
