#include "pspcom.h"

#include "FreeRTOS.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32h7xx_hal.h"
#include "string.h"
#include "task.h"
#include "timer.h"

#define PSPCOM_DEVICE_ID 1

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart7_tx;

uint16_t crc(uint16_t checksum, pspcommsg msg) {
    // Some code is taken from ChatGPT
    uint8_t *msg_content = (uint8_t *)malloc(5 + msg.payload_len);
    msg_content[0] = '!';
    msg_content[1] = '$';
    msg_content[2] = (uint8_t)msg.payload_len;
    msg_content[3] = (uint8_t)msg.device_id;
    msg_content[4] = (uint8_t)msg.msg_id;
    for (int i = 0; i < msg.payload_len; i++) {
        msg_content[5 + i] = msg.payload[i];
    }
    for (int i = 0; i < 5 + msg.payload_len; i++) {
        checksum ^= (uint16_t)msg_content[i] << 8;  // XOR with next byte
        for (int j = 0; j < 8; j++) {
            if (checksum & 0x8000) {
                checksum = (checksum << 1) ^ CRC16_POLY;
            } else {
                checksum <<= 1;
            }
        }
    }
    free(msg_content);
    return checksum;
}

Status pspcom_init() {
    HAL_NVIC_SetPriority(UART7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    huart7.Instance = UART7;
    huart7.Init.BaudRate = 115200;
    huart7.Init.WordLength = UART_WORDLENGTH_8B;
    huart7.Init.StopBits = UART_STOPBITS_1;
    huart7.Init.Parity = UART_PARITY_NONE;
    huart7.Init.Mode = UART_MODE_TX_RX;
    huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart7.Init.OverSampling = UART_OVERSAMPLING_16;
    huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart7) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK) {
        return STATUS_ERROR;
    }

    /* UART7 DMA Init */
    /* UART7_RX Init */
    hdma_uart7_rx.Instance = DMA1_Stream0;
    hdma_uart7_rx.Init.Request = DMA_REQUEST_UART7_RX;
    hdma_uart7_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart7_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart7_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart7_rx.Init.Mode = DMA_NORMAL;
    hdma_uart7_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_uart7_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart7_rx) != HAL_OK) {
        return STATUS_ERROR;
    }

    __HAL_LINKDMA(&huart7, hdmarx, hdma_uart7_rx);

    /* UART7_TX Init */
    hdma_uart7_tx.Instance = DMA1_Stream1;
    hdma_uart7_tx.Init.Request = DMA_REQUEST_UART7_TX;
    hdma_uart7_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart7_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart7_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart7_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart7_tx.Init.Mode = DMA_NORMAL;
    hdma_uart7_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_uart7_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart7_tx) != HAL_OK) {
        return STATUS_ERROR;
    }

    __HAL_LINKDMA(&huart7, hdmatx, hdma_uart7_tx);

    return STATUS_OK;
}

void pspcom_process_bytes(char *buf, int len) {}

void pspcom_send_msg(pspcommsg msg) {
    uint16_t checksum = crc(CRC16_INIT, msg);
    char *buf = (char *)malloc(7 + msg.payload_len);

    sprintf(buf, "!$%c%c%c", msg.payload_len, msg.device_id, msg.msg_id);
    memcpy(buf + 5, msg.payload, msg.payload_len);
    sprintf(buf + 5 + msg.payload_len, "%c%c", (uint8_t)checksum,
            (uint8_t)(checksum >> 8));
    HAL_UART_Transmit(&huart7, (uint8_t *)buf, 7 + msg.payload_len, 50);
    free(buf);
}

void pspcom_send_sensor(SensorFrame *sens) {
    vTaskSuspendAll();

    // Accelerometer
    pspcommsg tx_msg = {
        .payload_len = 13,
        .device_id = PSPCOM_DEVICE_ID,
        .msg_id = ACCEL,
    };
    tx_msg.payload[0] = 0;
    memcpy(tx_msg.payload + 1, &sens->acc_i_x, sizeof(float));
    memcpy(tx_msg.payload + 5, &sens->acc_i_y, sizeof(float));
    memcpy(tx_msg.payload + 9, &sens->acc_i_z, sizeof(float));
    pspcom_send_msg(tx_msg);

    // Gyroscope
    tx_msg.msg_id = GYRO;
    memcpy(tx_msg.payload + 1, &sens->rot_i_x, sizeof(float));
    memcpy(tx_msg.payload + 5, &sens->rot_i_y, sizeof(float));
    memcpy(tx_msg.payload + 9, &sens->rot_i_z, sizeof(float));
    pspcom_send_msg(tx_msg);

    // Temperature
    tx_msg.msg_id = TEMP;
    tx_msg.payload_len = 5;
    memcpy(tx_msg.payload + 1, &sens->temperature, sizeof(float));
    pspcom_send_msg(tx_msg);

    // Pressure
    tx_msg.msg_id = PRES;
    memcpy(tx_msg.payload + 1, &sens->pressure, sizeof(float));
    pspcom_send_msg(tx_msg);

    xTaskResumeAll();
}

void pspcom_send_gps(GPS_Fix_TypeDef *gps) {
    vTaskSuspendAll();

    // Position
    pspcommsg tx_msg = {
        .payload_len = 13,
        .device_id = PSPCOM_DEVICE_ID,
        .msg_id = GPS_POS,
    };
    tx_msg.payload[0] = gps->num_sats;
    memcpy(tx_msg.payload + 1, &gps->lat, sizeof(float));
    memcpy(tx_msg.payload + 5, &gps->lon, sizeof(float));
    memcpy(tx_msg.payload + 9, &gps->height_msl, sizeof(float));
    pspcom_send_msg(tx_msg);

    // Velocity
    tx_msg.msg_id = GPS_VEL;
    memcpy(tx_msg.payload + 1, &gps->vel_north, sizeof(float));
    memcpy(tx_msg.payload + 5, &gps->vel_east, sizeof(float));
    memcpy(tx_msg.payload + 9, &gps->vel_down, sizeof(float));
    pspcom_send_msg(tx_msg);

    xTaskResumeAll();
}

void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_uart7_rx); }

void DMA1_Stream1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_uart7_tx); }

void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&huart7); }