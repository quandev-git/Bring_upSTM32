#include "rc522.h"

#include <stdbool.h>

/* MFRC522 register addresses from the NXP datasheet. */
#define RC522_COMMAND_REG      0x01U
#define RC522_COM_IRQ_REG      0x04U
#define RC522_ERROR_REG        0x06U
#define RC522_FIFO_DATA_REG    0x09U
#define RC522_FIFO_LEVEL_REG   0x0AU
#define RC522_CONTROL_REG      0x0CU
#define RC522_BIT_FRAMING_REG  0x0DU
#define RC522_MODE_REG         0x11U
#define RC522_TX_CONTROL_REG   0x14U
#define RC522_TX_ASK_REG       0x15U
#define RC522_T_MODE_REG       0x2AU
#define RC522_T_PRESCALER_REG  0x2BU
#define RC522_T_RELOAD_H_REG   0x2CU
#define RC522_T_RELOAD_L_REG   0x2DU
#define RC522_VERSION_REG      0x37U

#define RC522_CMD_IDLE        0x00U
#define RC522_CMD_TRANSCEIVE  0x0CU
#define RC522_CMD_SOFT_RESET  0x0FU
#define RC522_PICC_WUPA       0x52U
#define RC522_PICC_HLTA       0x50U

static SPI_HandleTypeDef *rc522_spi;

static HAL_StatusTypeDef rc522_exchange(uint8_t address, uint8_t value, uint8_t *received)
{
    uint8_t tx[2] = {address, value};
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(rc522_spi, tx, rx, 2, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    if (received != NULL) *received = rx[1];
    return status;
}

static HAL_StatusTypeDef rc522_write(uint8_t reg, uint8_t value)
{
    return rc522_exchange((reg << 1) & 0x7EU, value, NULL);
}

static uint8_t rc522_read(uint8_t reg)
{
    uint8_t value = 0;
    (void)rc522_exchange(((reg << 1) & 0x7EU) | 0x80U, 0, &value);
    return value;
}

HAL_StatusTypeDef RC522_Init(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL) return HAL_ERROR;
    rc522_spi = hspi;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(50);

    if (rc522_write(RC522_COMMAND_REG, RC522_CMD_SOFT_RESET) != HAL_OK) return HAL_ERROR;
    HAL_Delay(50);
    uint8_t version = rc522_read(RC522_VERSION_REG);
    if (version == 0x00U || version == 0xFFU) return HAL_ERROR;

    if (rc522_write(RC522_T_MODE_REG, 0x8DU) != HAL_OK ||
        rc522_write(RC522_T_PRESCALER_REG, 0x3EU) != HAL_OK ||
        rc522_write(RC522_T_RELOAD_H_REG, 0x00U) != HAL_OK ||
        rc522_write(RC522_T_RELOAD_L_REG, 30U) != HAL_OK ||
        rc522_write(RC522_TX_ASK_REG, 0x40U) != HAL_OK ||
        rc522_write(RC522_MODE_REG, 0x3DU) != HAL_OK) return HAL_ERROR;

    return rc522_write(RC522_TX_CONTROL_REG, rc522_read(RC522_TX_CONTROL_REG) | 0x03U);
}

static uint16_t rc522_crc_a(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0x6363U;
    for (uint8_t i = 0; i < length; ++i) {
        uint8_t value = data[i] ^ (uint8_t)crc;
        value ^= (uint8_t)(value << 4);
        crc = (crc >> 8) ^ ((uint16_t)value << 8) ^
              ((uint16_t)value << 3) ^ ((uint16_t)value >> 4);
    }
    return crc;
}

static bool rc522_transceive(const uint8_t *tx, uint8_t tx_length,
                             uint8_t tx_last_bits, uint8_t *rx,
                             uint8_t *rx_length, uint8_t capacity)
{
    if (rc522_spi == NULL || tx == NULL || tx_length == 0U ||
        rx_length == NULL) return false;

    if (rc522_write(RC522_COMMAND_REG, RC522_CMD_IDLE) != HAL_OK ||
        rc522_write(RC522_COM_IRQ_REG, 0x7FU) != HAL_OK ||
        rc522_write(RC522_FIFO_LEVEL_REG, 0x80U) != HAL_OK) return false;
    for (uint8_t i = 0; i < tx_length; ++i) {
        if (rc522_write(RC522_FIFO_DATA_REG, tx[i]) != HAL_OK) return false;
    }
    if (rc522_write(RC522_BIT_FRAMING_REG, tx_last_bits) != HAL_OK ||
        rc522_write(RC522_COMMAND_REG, RC522_CMD_TRANSCEIVE) != HAL_OK ||
        rc522_write(RC522_BIT_FRAMING_REG, tx_last_bits | 0x80U) != HAL_OK)
        return false;

    uint32_t start = HAL_GetTick();
    uint8_t irq = 0;
    do {
        irq = rc522_read(RC522_COM_IRQ_REG);
        if (irq & 0x31U) break; /* RxIRQ, IdleIRQ, or timer timeout. */
    } while ((HAL_GetTick() - start) < 25U);

    (void)rc522_write(RC522_BIT_FRAMING_REG, tx_last_bits);
    if (!(irq & 0x30U) || (rc522_read(RC522_ERROR_REG) & 0x1BU)) return false;
    uint8_t count = rc522_read(RC522_FIFO_LEVEL_REG);
    if (count > capacity || (count != 0U && rx == NULL)) return false;
    for (uint8_t i = 0; i < count; ++i) rx[i] = rc522_read(RC522_FIFO_DATA_REG);
    *rx_length = count;
    return true;
}

static void rc522_halt(void)
{
    uint8_t command[4] = {RC522_PICC_HLTA, 0x00U, 0, 0};
    uint16_t crc = rc522_crc_a(command, 2);
    command[2] = (uint8_t)crc;
    command[3] = (uint8_t)(crc >> 8);
    uint8_t ignored = 0;
    /* HLTA has no reply when successful. */
    (void)rc522_transceive(command, 4, 0, NULL, &ignored, 0);
}

bool RC522_ReadUid(rc522_uid_t *uid)
{
    if (uid == NULL) return false;
    *uid = (rc522_uid_t){0};

    uint8_t atqa[2];
    uint8_t received = 0;
    const uint8_t request = RC522_PICC_WUPA;
    if (!rc522_transceive(&request, 1, 7, atqa, &received, sizeof(atqa)) ||
        received != 2U || (rc522_read(RC522_CONTROL_REG) & 0x07U) != 0U)
        return false;

    static const uint8_t cascade_codes[3] = {0x93U, 0x95U, 0x97U};
    for (uint8_t level = 0; level < 3U; ++level) {
        uint8_t anticollision[2] = {cascade_codes[level], 0x20U};
        uint8_t block[5];
        if (!rc522_transceive(anticollision, 2, 0, block, &received,
                             sizeof(block)) || received != 5U ||
            (rc522_read(RC522_CONTROL_REG) & 0x07U) != 0U ||
            (uint8_t)(block[0] ^ block[1] ^ block[2] ^ block[3]) != block[4])
            return false;

        uint8_t select[9] = {cascade_codes[level], 0x70U};
        for (uint8_t i = 0; i < 5U; ++i) select[i + 2U] = block[i];
        uint16_t crc = rc522_crc_a(select, 7);
        select[7] = (uint8_t)crc;
        select[8] = (uint8_t)(crc >> 8);
        uint8_t sak[3];
        if (!rc522_transceive(select, 9, 0, sak, &received, sizeof(sak)) ||
            received != 3U || (rc522_read(RC522_CONTROL_REG) & 0x07U) != 0U ||
            rc522_crc_a(sak, 1) != ((uint16_t)sak[1] | ((uint16_t)sak[2] << 8)))
            return false;

        bool more_levels = (sak[0] & 0x04U) != 0U;
        if (more_levels != (block[0] == 0x88U)) return false;
        uint8_t first = more_levels ? 1U : 0U;
        uint8_t count = more_levels ? 3U : 4U;
        if ((uint8_t)(uid->length + count) > sizeof(uid->bytes)) return false;
        for (uint8_t i = 0; i < count; ++i)
            uid->bytes[uid->length++] = block[first + i];

        if (!more_levels) {
            rc522_halt();
            return true;
        }
    }
    return false;
}

bool RC522_CardPresent(void)
{
    rc522_uid_t uid;
    return RC522_ReadUid(&uid);
}
