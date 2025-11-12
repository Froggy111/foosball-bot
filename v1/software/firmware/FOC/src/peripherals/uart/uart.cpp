#include "uart.hpp"

#include <stm32g4xx_hal.h>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"

static UART_HandleTypeDef uart_handle;
static DMA_HandleTypeDef tx_dma_handle;
static DMA_HandleTypeDef rx_dma_handle;

static volatile bool transmission_completed = true;

void uart::init(uint32_t baud_rate) {
    // initialise GPIOs
    gpio::init(UART_TX, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);
    gpio::init(UART_RX, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);
#ifdef USE_RS485
    gpio::init(UART_DE, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);
#endif

    // initialise DMA
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    tx_dma_handle.Instance = UART_TX_DMA_INSTANCE;
    tx_dma_handle.Init.Request = DMA_REQUEST_USART1_TX;
    tx_dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    tx_dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    tx_dma_handle.Init.MemInc = DMA_MINC_ENABLE;
    tx_dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    tx_dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    tx_dma_handle.Init.Mode = DMA_NORMAL;
    tx_dma_handle.Init.Priority = DMA_PRIORITY_LOW;

    rx_dma_handle.Instance = UART_RX_DMA_INSTANCE;
    rx_dma_handle.Init.Request = DMA_REQUEST_USART1_RX;
    rx_dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    rx_dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    rx_dma_handle.Init.MemInc = DMA_MINC_ENABLE;
    rx_dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    rx_dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    rx_dma_handle.Init.Mode = DMA_NORMAL;
    rx_dma_handle.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&tx_dma_handle) != HAL_OK) {
        debug::error("UART TX DMA initialisation failed");
        error::handler();
    }
    if (HAL_DMA_Init(&rx_dma_handle) != HAL_OK) {
        debug::error("UART RX DMA initialisation failed");
        error::handler();
    }
    __HAL_LINKDMA(&uart_handle, hdmatx, tx_dma_handle);
    __HAL_LINKDMA(&uart_handle, hdmarx, rx_dma_handle);

    // initialise UART
    if (UART_INSTANCE == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
    }
    uart_handle.Instance = UART_INSTANCE;
    uart_handle.Init.BaudRate = baud_rate;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle.Init.StopBits = UART_STOPBITS_1;
    uart_handle.Init.Parity = UART_PARITY_NONE;
    uart_handle.Init.Mode = UART_MODE_TX_RX;
    uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&uart_handle) != HAL_OK) {
        debug::error("UART initialisation failed");
        error::handler();
    }
#ifdef USE_RS485
    if (HAL_RS485Ex_Init(&uart_handle, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK) {
        debug::error("RS485 initialisation failed");
        error::handler();
    }
#endif

    // initialise interrupts
    // allow ADC and more important to preempt
    if (UART_TX_DMA_INSTANCE == DMA1_Channel3) {
        HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    }
    if (UART_RX_DMA_INSTANCE == DMA1_Channel4) {
        HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    }
    if (UART_INSTANCE == USART1) {
        HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    return;
}

void uart::transmit(uint8_t *data, uint16_t len) {
    if (transmission_completed) {
        transmission_completed = false;
        HAL_UART_Transmit_DMA(&uart_handle, data, len);
    }
    return;
}

bool uart::can_transmit(void) { return transmission_completed; }

void uart::transmit_complete_callback(void) { transmission_completed = true; }

void uart::receive_complete_callback(void) {}
