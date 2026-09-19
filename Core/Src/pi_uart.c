#include "pi_uart.h"

#include "queue.h"

#define PI_UART_BAUD 115200U

static QueueHandle_t pi_rx_queue;

HAL_StatusTypeDef PiUart_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    pi_rx_queue = xQueueCreate(64, sizeof(uint8_t));
    if (pi_rx_queue == NULL) return HAL_ERROR;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = (HAL_RCC_GetPCLK2Freq() + PI_UART_BAUD / 2U) /
                  PI_UART_BAUD;
    (void)USART1->SR;
    (void)USART1->DR;
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
    return HAL_OK;
}

void PiUart_StartRx(void)
{
    USART1->CR1 |= USART_CR1_RXNEIE;
}

BaseType_t PiUart_ReadByte(uint8_t *byte, TickType_t wait_ticks)
{
    return xQueueReceive(pi_rx_queue, byte, wait_ticks);
}

void USART1_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE |
                   USART_SR_FE | USART_SR_PE)) == 0U) return;
    uint8_t byte = (uint8_t)USART1->DR; /* SR then DR clears RX and errors. */
    if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) == 0U &&
        (status & USART_SR_RXNE) != 0U && pi_rx_queue != NULL) {
        BaseType_t woke_task = pdFALSE;
        (void)xQueueSendFromISR(pi_rx_queue, &byte, &woke_task);
        portYIELD_FROM_ISR(woke_task);
    }
}
