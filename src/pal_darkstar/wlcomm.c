#include "wlcomm.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "timer.h"
#include "timers.h"

#define UART_BUFFER_SIZE 512

#define DMA_WRITE_PTR                                         \
    ((UART_BUFFER_SIZE -                                      \
      ((DMA_Stream_TypeDef*)huart9.hdmarx->Instance)->NDTR) & \
     (UART_BUFFER_SIZE - 1))

RAM_D2 struct {
    uint8_t buffer[UART_BUFFER_SIZE];
    size_t rd_ptr;
} cb;

UART_HandleTypeDef huart9;
DMA_HandleTypeDef hdma_uart9_rx;
DMA_HandleTypeDef hdma_uart9_tx;
TimerHandle_t arm_timer;

static uint32_t s_global_nack = 0;
static uint32_t s_global_ack = 0;

static Status start_uart_reading();
static void circular_buffer_init();
static uint8_t circular_buffer_is_empty();
static Status circular_buffer_pop(uint8_t* data);

Status wlcomm_init() {
    __HAL_RCC_UART9_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(UART9_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(UART9_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_UART9;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    huart9.Instance = UART9;
    huart9.Init.BaudRate = 115200;
    huart9.Init.WordLength = UART_WORDLENGTH_8B;
    huart9.Init.StopBits = UART_STOPBITS_1;
    huart9.Init.Parity = UART_PARITY_NONE;
    huart9.Init.Mode = UART_MODE_TX_RX;
    huart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart9.Init.OverSampling = UART_OVERSAMPLING_16;
    huart9.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart9.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart9.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart9) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart9, UART_TXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart9, UART_RXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_UARTEx_DisableFifoMode(&huart9) != HAL_OK) {
        return STATUS_ERROR;
    }

    /* UART9 DMA Init */
    /* UART9_RX Init */
    hdma_uart9_rx.Instance = DMA1_Stream0;
    hdma_uart9_rx.Init.Request = DMA_REQUEST_UART9_RX;
    hdma_uart9_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart9_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart9_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart9_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart9_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart9_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart9_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart9_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart9_rx) != HAL_OK) {
        return STATUS_ERROR;
    }

    __HAL_LINKDMA(&huart9, hdmarx, hdma_uart9_rx);

    /* UART9_TX Init */
    hdma_uart9_tx.Instance = DMA1_Stream1;
    hdma_uart9_tx.Init.Request = DMA_REQUEST_UART9_TX;
    hdma_uart9_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart9_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart9_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart9_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart9_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart9_tx.Init.Mode = DMA_NORMAL;
    hdma_uart9_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_uart9_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart9_tx) != HAL_OK) {
        return STATUS_ERROR;
    }

    __HAL_LINKDMA(&huart9, hdmatx, hdma_uart9_tx);

    circular_buffer_init();
    ASSERT_OK(start_uart_reading(), "wlcomm UART read\n");

    return STATUS_OK;
}

Status wlcomm_recv_msg(pspcommsg* msg) {
    uint8_t current_byte = 0;
    static uint8_t state = 0;
    static uint16_t checksum;
    static uint8_t payload_ctr = 0;

    while (!circular_buffer_is_empty()) {
        switch (state) {
            case 0:
                memset(msg, 0, sizeof(pspcommsg));
                payload_ctr = 0;
                checksum = 0;
                circular_buffer_pop(&current_byte);
                if (current_byte == '!') {
                    state = 1;
                }
                break;
            case 1:
                circular_buffer_pop(&current_byte);
                if (current_byte == '$') {
                    state = 2;
                } else {
                    state = 0;
                }
                break;
            case 2:
                circular_buffer_pop(&current_byte);
                msg->payload_len = current_byte;
                state = 3;
                break;
            case 3:
                circular_buffer_pop(&current_byte);
                msg->device_id = current_byte;
                state = 4;
                break;
            case 4:
                circular_buffer_pop(&current_byte);
                msg->msg_id = current_byte;
                state = 5;
                break;
            case 5:
                if (payload_ctr < msg->payload_len) {
                    circular_buffer_pop(&current_byte);
                    msg->payload[payload_ctr] = current_byte;
                    payload_ctr++;
                    break;
                } else {
                    state = 6;
                }
            case 6:
                circular_buffer_pop(&current_byte);
                checksum = current_byte;
                state = 7;
                break;
            case 7:
                circular_buffer_pop(&current_byte);
                checksum += ((uint16_t)current_byte) << 8;
                if (crc16(CRC16_INIT, msg) == checksum) {
                    state = 0;
                    return STATUS_OK;
                }
                state = 0;
                break;
        }
    }

    return STATUS_ERROR;
}

Status wlcomm_send_msg(pspcommsg* msg) {
    uint16_t checksum = crc16(CRC16_INIT, msg);
    char buf[7 + msg->payload_len];

    sprintf(buf, "!$%c%c%c", msg->payload_len, msg->device_id, msg->msg_id);
    memcpy(buf + 5, msg->payload, msg->payload_len);
    sprintf(buf + 5 + msg->payload_len, "%c%c", (uint8_t)checksum,
            (uint8_t)(checksum >> 8));

    HAL_UART_Transmit_DMA(&huart9, (uint8_t*)buf, 7 + msg->payload_len);

    uint64_t timeout = MILLIS();
    while (HAL_UART_GetState(&huart9) == HAL_UART_STATE_BUSY_TX) {
        if (MILLIS() - timeout > 100) {
            break;
        }
        DELAY(1);
    }

    return STATUS_OK;
}

Status wlcomm_set_freq(uint32_t frequency_hz) {
    s_global_ack = 0;
    s_global_nack = 0;

    pspcommsg msg = {
        .payload_len = 5,
        .device_id = PSPCOM_DEVICE_ID,
        .msg_id = SET_LOCAL_FREQ,
    };

    // Radio 0
    msg.payload[0] = 0;

    // Frequency
    memcpy(msg.payload + 1, &frequency_hz, sizeof(uint32_t));
    wlcomm_send_msg(&msg);

    uint64_t timeout = MILLIS();
    while (MILLIS() - timeout < 10000) {
        DELAY(500);

        if (s_global_ack) {
            s_global_ack = 0;
            s_global_nack = 0;
            return STATUS_OK;
        } else if (s_global_nack) {
            s_global_ack = 0;
            s_global_nack = 0;
            return STATUS_ERROR;
        }

        wlcomm_send_msg(&msg);
    }

    return STATUS_ERROR;
}

static Status start_uart_reading() {
    if (HAL_UART_Receive_DMA(&huart9, cb.buffer, UART_BUFFER_SIZE) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

static void circular_buffer_init() { cb.rd_ptr = 0; }

static uint8_t circular_buffer_is_empty() {
    if (cb.rd_ptr == DMA_WRITE_PTR) {
        return 1;
    }
    return 0;
}

static Status circular_buffer_pop(uint8_t* data) {
    if (cb.rd_ptr != DMA_WRITE_PTR) {
        *data = cb.buffer[cb.rd_ptr++];
        cb.rd_ptr %= UART_BUFFER_SIZE;
        return STATUS_OK;
    } else {
        return STATUS_ERROR;
    }
}
