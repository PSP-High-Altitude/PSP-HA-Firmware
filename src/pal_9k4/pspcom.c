#include "pspcom.h"

#include "FreeRTOS.h"
#include "gpio/gpio.h"
#include "main.h"
#include "pal_9k31/FreeRTOS/Source/include/timers.h"
#include "sensors.h"
#include "state.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32h7xx_hal.h"
#include "string.h"
#include "task.h"
#include "timer.h"

#define PSPCOM_DEVICE_ID 1

#define SENSOR_TELEM_PERIOD 5000
#define GPS_TELEM_PERIOD 5000
#define STATUS_TELEM_PERIOD 5000
#define STD_TELEM_PERIOD 5000

#define PYRO_RETRIES_MAN 3
#define PYRO_RETRY_PERIOD_MAN 1000
#define PYRO_FIRE_PERIOD_MAN 1000

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart7_tx;
TimerHandle_t arm_timer;

uint8_t user_armed[3] = {0, 0, 0};

#define UART_BUFFER_SIZE 512

#define DMA_WRITE_PTR                                          \
    ((UART_BUFFER_SIZE -                                       \
      ((DMA_Stream_TypeDef *)huart7.hdmarx->Instance)->NDTR) & \
     (UART_BUFFER_SIZE - 1))

struct {
    uint8_t buffer[UART_BUFFER_SIZE];
    size_t rd_ptr;
} cb;

static Status start_uart_reading();
// static Status stop_uart_reading();
static void circular_buffer_init();
static uint8_t circular_buffer_is_empty();
static Status circular_buffer_pop(uint8_t *data);
static Status pspcom_read_msg_from_uart(pspcommsg *msg);
static Status start_arm_timeout(uint32_t pyro);
static void arm_timeout_callback(TimerHandle_t xTimer);

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
    hdma_uart7_rx.Init.Mode = DMA_CIRCULAR;
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

    circular_buffer_init();
    if (start_uart_reading() != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Initialize arm timeout timer
    arm_timer = xTimerCreate("ArmTimer", 1000 / portTICK_PERIOD_MS, pdFALSE,
                             (void *)0, arm_timeout_callback);

    return STATUS_OK;
}

Status pspcom_read_msg_from_uart(pspcommsg *msg) {
    static uint8_t state = 0;
    uint8_t current_byte = 0;
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
                if (crc(CRC16_INIT, *msg) == checksum) {
                    state = 0;
                    return STATUS_OK;
                }
                state = 0;
                break;
        }
    }
    return STATUS_ERROR;
}

void pspcom_process_bytes(char *buf, int len) {
    pspcommsg msg;
    while (1) {
        if (pspcom_read_msg_from_uart(&msg) == STATUS_OK) {
#ifdef DEBUG
            printf("Received message: %d %d %d\n", msg.payload_len,
                   msg.device_id, msg.msg_id);
#endif
            switch (msg.msg_id) {
                case ARMMAIN:
                case ARMDRG:
                case ARMAUX:
                    user_armed[msg.msg_id - ARMMAIN] = 1;
                    if (start_arm_timeout(msg.msg_id - ARMMAIN) != STATUS_OK) {
#ifdef DEBUG
                        printf("Error starting arm timeout\n");
#endif
                        user_armed[msg.msg_id - ARMMAIN] = 0;
                    }
                    break;
                case FIREMAIN:
                    if (user_armed[0]) {
                        for (int i = 0; i < PYRO_RETRIES_MAN; i++) {
                            gpio_write(PIN_FIREMAIN, 1);
                            vTaskDelay(PYRO_FIRE_PERIOD_MAN /
                                       portTICK_PERIOD_MS);
                            gpio_write(PIN_FIREMAIN, 0);
                            vTaskDelay(PYRO_RETRY_PERIOD_MAN /
                                       portTICK_PERIOD_MS);
                            if (gpio_read(PIN_CONTMAIN) == 0) {
                                break;
                            }
                        }
                    }
                case FIREDRG:
                    if (user_armed[1]) {
                        for (int i = 0; i < PYRO_RETRIES_MAN; i++) {
                            gpio_write(PIN_FIREDRG, 1);
                            vTaskDelay(PYRO_FIRE_PERIOD_MAN /
                                       portTICK_PERIOD_MS);
                            gpio_write(PIN_FIREDRG, 0);
                            vTaskDelay(PYRO_RETRY_PERIOD_MAN /
                                       portTICK_PERIOD_MS);
                            if (gpio_read(PIN_CONTDRG) == 0) {
                                break;
                            }
                        }
                    }
                case FIREAUX:
                    if (user_armed[2]) {
                        for (int i = 0; i < PYRO_RETRIES_MAN; i++) {
                            gpio_write(PIN_FIREAUX, 1);
                            vTaskDelay(PYRO_FIRE_PERIOD_MAN /
                                       portTICK_PERIOD_MS);
                            gpio_write(PIN_FIREAUX, 0);
                            vTaskDelay(PYRO_RETRY_PERIOD_MAN /
                                       portTICK_PERIOD_MS);
                            if (gpio_read(PIN_CONTAUX) == 0) {
                                break;
                            }
                        }
                    }
                    break;
                default:
                    break;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void pspcom_send_msg(pspcommsg msg) {
    uint16_t checksum = crc(CRC16_INIT, msg);
    char *buf = (char *)malloc(7 + msg.payload_len);

    sprintf(buf, "!$%c%c%c", msg.payload_len, msg.device_id, msg.msg_id);
    memcpy(buf + 5, msg.payload, msg.payload_len);
    sprintf(buf + 5 + msg.payload_len, "%c%c", (uint8_t)checksum,
            (uint8_t)(checksum >> 8));
    HAL_UART_Transmit_DMA(&huart7, (uint8_t *)buf, 7 + msg.payload_len);
    uint64_t timeout = MILLIS();
    while (HAL_UART_GetState(&huart7) == HAL_UART_STATE_BUSY_TX) {
        if (MILLIS() - timeout > 100) {
            break;
        }
        vTaskDelay(1);
    }
    free(buf);
}

void pspcom_send_sensor(void *sensor_frame) {
    SensorFrame *sens = (SensorFrame *)sensor_frame;
    while (1) {
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

        vTaskDelay(SENSOR_TELEM_PERIOD / portTICK_PERIOD_MS);
    }
}

void pspcom_send_gps(void *gps_fix) {
    GPS_Fix_TypeDef *gps = (GPS_Fix_TypeDef *)gps_fix;
    while (1) {
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

        vTaskDelay(GPS_TELEM_PERIOD / portTICK_PERIOD_MS);
    }
}

void pspcom_send_status() {
    while (1) {
        // Pyro Status
        pspcommsg tx_msg = {
            .payload_len = 1,
            .device_id = PSPCOM_DEVICE_ID,
            .msg_id = PYRO_STAT,
        };
        uint8_t main_cont = gpio_read(PIN_CONTMAIN);
        uint8_t drg_cont = gpio_read(PIN_CONTDRG);
        uint8_t aux_cont = gpio_read(PIN_CONTAUX);
        tx_msg.payload[0] =
            (main_cont << 1) | (drg_cont << 3) | (aux_cont << 5) | 0x15;
        pspcom_send_msg(tx_msg);

        vTaskDelay(STATUS_TELEM_PERIOD / portTICK_PERIOD_MS);
    }
}

void pspcom_send_standard() {
    TickType_t last_standard_tx_ticks = xTaskGetTickCount();

    while (1) {
        // Get pointers to latest data
        SensorFrame *sensor_frame = get_last_sensor_frame();
        GPS_Fix_TypeDef *gps_fix = get_last_gps_fix();
        FlightPhase *flight_phase = get_last_flight_phase();

        // Standard telemetry
        pspcommsg tx_msg = {
            .payload_len = 18,
            .device_id = PSPCOM_DEVICE_ID,
            .msg_id = STD_TELEM_1,
        };

        // GPS_POS
        gps_pos_packed gps_pos;
        gps_pos.num_sats = gps_fix->num_sats & 0x1F;
        gps_pos.lat = ((int32_t)(gps_fix->lat / 0.0000108)) & 0x00FFFFFF;
        gps_pos.lon = ((int32_t)(gps_fix->lon / 0.0000108)) & 0x01FFFFFF;
        gps_pos.alt = ((uint32_t)(gps_fix->height_msl + 1000)) & 0x0003FFFF;
        memcpy(tx_msg.payload, &gps_pos, sizeof(gps_pos_packed));

        // GPS_VEL
        gps_vel_packed gps_vel;
        gps_vel.veln = ((int16_t)(gps_fix->vel_north)) & 0x1FFF;
        gps_vel.vele = ((int16_t)(gps_fix->vel_east)) & 0x1FFF;
        gps_vel.veld = ((int16_t)(gps_fix->vel_down)) & 0x3FFF;
        memcpy(tx_msg.payload + 9, &gps_vel, sizeof(gps_vel_packed));

        // PRES
        uint16_t pres = (uint16_t)(sensor_frame->pressure / 0.025);
        tx_msg.payload[14] = pres & 0xFF;
        tx_msg.payload[15] = (pres >> 8) & 0xFF;

        // PYRO_STAT
        uint8_t main_cont = gpio_read(PIN_CONTMAIN);
        uint8_t drg_cont = gpio_read(PIN_CONTDRG);
        uint8_t aux_cont = gpio_read(PIN_CONTAUX);
        tx_msg.payload[16] =
            (main_cont << 1) | (drg_cont << 3) | (aux_cont << 5) | 0x15;

        // SYS_STAT
        tx_msg.payload[17] = (uint8_t)*flight_phase & 0x1;
        tx_msg.payload[17] = ((uint8_t)*flight_phase & 0xF) << 3;

        pspcom_send_msg(tx_msg);

        vTaskDelayUntil(&last_standard_tx_ticks,
                        pdMS_TO_TICKS(STD_TELEM_PERIOD));
    }
}

static Status start_uart_reading() {
    if (HAL_UART_Receive_DMA(&huart7, cb.buffer, UART_BUFFER_SIZE) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

/*
static Status stop_uart_reading() {
    if (HAL_UART_AbortReceive(&huart7) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}
*/

static void circular_buffer_init() { cb.rd_ptr = 0; }

static uint8_t circular_buffer_is_empty() {
    if (cb.rd_ptr == DMA_WRITE_PTR) {
        return 1;
    }
    return 0;
}

static Status circular_buffer_pop(uint8_t *data) {
    if (cb.rd_ptr != DMA_WRITE_PTR) {
        *data = cb.buffer[cb.rd_ptr++];
        cb.rd_ptr %= UART_BUFFER_SIZE;
        return STATUS_OK;
    } else {
        return STATUS_ERROR;
    }
}

static Status start_arm_timeout(uint32_t pyro) {
    if (xTimerIsTimerActive(arm_timer) == pdTRUE) {
        return STATUS_ERROR;
    }
    vTimerSetTimerID(arm_timer, (void *)pyro);
    if (xTimerStart(arm_timer, 0) != pdPASS) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}
static void arm_timeout_callback(TimerHandle_t xTimer) {
    uint32_t pyro = (uint32_t)pvTimerGetTimerID(xTimer);
#ifdef DEBUG
    printf("Pyro %lu timeout\n", pyro);
#endif
    switch (pyro) {
        case 0:
            user_armed[0] = 0;
            break;
        case 1:
            user_armed[1] = 0;
            break;
        case 2:
            user_armed[2] = 0;
            break;
    }
}

void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_uart7_rx); }

void DMA1_Stream1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_uart7_tx); }

void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&huart7); }