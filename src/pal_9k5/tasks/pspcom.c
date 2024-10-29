#include "pspcom.h"

#include "FreeRTOS.h"
#include "backup/backup.h"
#include "flight_control.h"
#include "gpio/gpio.h"
#include "main.h"
#include "pyros.h"
#include "queue.h"
#include "sensors.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32h7xx_hal.h"
#include "string.h"
#include "task.h"
#include "timer.h"
#include "timers.h"

#define PSPCOM_DEVICE_ID 0x10  // PAL 9k5

#define SENSOR_TELEM_PERIOD 5000
#define GPS_TELEM_PERIOD 5000
#define STATUS_TELEM_PERIOD 5000
#define STD_TELEM_PERIOD_GROUND 5000
#define STD_TELEM_PERIOD_FLIGHT 1000

#define PYRO_RETRIES_MAN 3
#define PYRO_RETRY_PERIOD_MAN 1000
#define PYRO_FIRE_PERIOD_MAN 1000

UART_HandleTypeDef huart9;
DMA_HandleTypeDef hdma_uart9_rx;
DMA_HandleTypeDef hdma_uart9_tx;
TimerHandle_t arm_timer;

static QueueHandle_t s_sensor_queue;
static QueueHandle_t s_gps_queue;
static QueueHandle_t s_fp_queue;

uint8_t user_armed[5] = {0, 0, 0, 0, 0};

#define UART_BUFFER_SIZE 512

#define DMA_WRITE_PTR                                          \
    ((UART_BUFFER_SIZE -                                       \
      ((DMA_Stream_TypeDef *)huart9.hdmarx->Instance)->NDTR) & \
     (UART_BUFFER_SIZE - 1))

RAM_D2 struct {
    uint8_t buffer[UART_BUFFER_SIZE];
    size_t rd_ptr;
} cb;

static BoardConfig *s_config_ptr = NULL;

static uint32_t s_global_nack = 0;
static uint32_t s_global_ack = 0;

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
    s_config_ptr = config_get_ptr();

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
    if (start_uart_reading() != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Initialize arm timeout timer
    arm_timer = xTimerCreate("ArmTimer", 1000 / portTICK_PERIOD_MS, pdFALSE,
                             (void *)0, arm_timeout_callback);

    // Queues for synchronization, not buffering
    s_sensor_queue = xQueueCreate(1, sizeof(SensorFrame));
    s_gps_queue = xQueueCreate(1, sizeof(GPS_Fix_TypeDef));
    s_fp_queue = xQueueCreate(1, sizeof(FlightPhase));

    configASSERT(s_sensor_queue);
    configASSERT(s_gps_queue);
    configASSERT(s_fp_queue);

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

void task_pspcom_rx() {
    pspcommsg msg;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        if (pspcom_read_msg_from_uart(&msg) == STATUS_OK) {
            PAL_LOGI("Received message: %d %d %d\n", msg.payload_len,
                     msg.device_id, msg.msg_id);
            switch (msg.msg_id) {
                case ACK:
                    s_global_ack = 1;
                    break;
                case NACK:
                    s_global_nack = 1;
                    break;
                case ARMMAIN:
                case ARMDRG:
                    user_armed[msg.msg_id - ARMMAIN] = 1;
                    if (start_arm_timeout(msg.msg_id - ARMMAIN) != STATUS_OK) {
                        user_armed[msg.msg_id - ARMMAIN] = 0;
                    }
                    break;
                case ARMAUX:
                    // For a 0-length payload, arm A1
                    if (msg.payload_len == 0) {
                        user_armed[PYRO_A1] = 1;
                        if (start_arm_timeout(PYRO_A1) != STATUS_OK) {
                            user_armed[PYRO_A1] = 0;
                        }
                    }  // Otherwise, determine the right AUX pyro to arm
                    else if (msg.payload_len == 1) {
                        if (msg.payload[0] == 0U) {
                            user_armed[PYRO_A1] = 1;
                            if (start_arm_timeout(PYRO_A1) != STATUS_OK) {
                                user_armed[PYRO_A1] = 0;
                            }
                        } else if (msg.payload[0] == 1U) {
                            user_armed[PYRO_A2] = 1;
                            if (start_arm_timeout(PYRO_A2) != STATUS_OK) {
                                user_armed[PYRO_A2] = 0;
                            }
                        } else if (msg.payload[0] == 2U) {
                            user_armed[PYRO_A3] = 1;
                            if (start_arm_timeout(PYRO_A3) != STATUS_OK) {
                                user_armed[PYRO_A3] = 0;
                            }
                        }
                    }
                    break;
                case FIREMAIN:
                    if (user_armed[0]) {
                        pyros_fire(PYRO_MAIN);
                    }
                    break;
                case FIREDRG:
                    if (user_armed[1]) {
                        pyros_fire(PYRO_DRG);
                    }
                    break;
                case FIREAUX:
                    // For a 0-length payload, fire A1
                    if (user_armed[2] && msg.payload_len == 0) {
                        pyros_fire(PYRO_A1);
                    }  // Otherwise, determine the right AUX pyro to fire
                    else if (msg.payload_len == 1) {
                        if (user_armed[2] && msg.payload[0] == 0U) {
                            pyros_fire(PYRO_A1);
                        } else if (user_armed[3] && msg.payload[0] == 1U) {
                            pyros_fire(PYRO_A2);
                        } else if (user_armed[4] && msg.payload[0] == 2U) {
                            pyros_fire(PYRO_A3);
                        }
                    }
                    break;
                default:
                    break;
            }
        }
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(s_config_ptr->pspcom_rx_loop_period_ms));
    }
}

void pspcom_send_msg(pspcommsg msg) {
    uint16_t checksum = crc(CRC16_INIT, msg);
    char buf[7 + msg.payload_len];

    sprintf(buf, "!$%c%c%c", msg.payload_len, msg.device_id, msg.msg_id);
    memcpy(buf + 5, msg.payload, msg.payload_len);
    sprintf(buf + 5 + msg.payload_len, "%c%c", (uint8_t)checksum,
            (uint8_t)(checksum >> 8));
    HAL_UART_Transmit_DMA(&huart9, (uint8_t *)buf, 7 + msg.payload_len);
    uint64_t timeout = MILLIS();
    while (HAL_UART_GetState(&huart9) == HAL_UART_STATE_BUSY_TX) {
        if (MILLIS() - timeout > 100) {
            break;
        }
        vTaskDelay(1);
    }
}

void pspcom_send_sensor(SensorFrame *sensor_frame) {
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

void pspcom_send_gps(GPS_Fix_TypeDef *gps_fix) {
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

void pspcom_update_sensors(SensorFrame *sensor_frame) {
    xQueueOverwrite(s_sensor_queue, sensor_frame);
}

void pspcom_update_gps(GPS_Fix_TypeDef *gps_fix) {
    xQueueOverwrite(s_gps_queue, gps_fix);
}

void pspcom_update_fp(FlightPhase flight_phase) {
    xQueueOverwrite(s_fp_queue, &flight_phase);
}

void pspcom_send_status() {
    while (1) {
        // Pyro Status
        pspcommsg tx_msg = {
            .payload_len = 2,
            .device_id = PSPCOM_DEVICE_ID,
            .msg_id = PYRO_STAT,
        };
        uint8_t main_cont = gpio_read(PIN_CONTMAIN);
        uint8_t drg_cont = gpio_read(PIN_CONTDRG);
        uint8_t a1_cont = gpio_read(PIN_CONTA1);
        uint8_t a2_cont = gpio_read(PIN_CONTA2);
        uint8_t a3_cont = gpio_read(PIN_CONTA3);
        // Show continuity and consider all armed
        tx_msg.payload[0] = (main_cont << 1) | (drg_cont << 3) |
                            (a1_cont << 5) | (a2_cont << 7) | 0x55;
        tx_msg.payload[1] = (a3_cont << 1) | 0x1;
        pspcom_send_msg(tx_msg);

        vTaskDelay(STATUS_TELEM_PERIOD / portTICK_PERIOD_MS);
    }
}

void pspcom_send_standard() {
    static SensorFrame s_sensor_frame;
    static GPS_Fix_TypeDef s_gps_fix;
    static FlightPhase s_flight_phase;

    // Get latest data
    if (xQueueReceive(s_sensor_queue, &s_sensor_frame, 0) != pdPASS) {
        PAL_LOGW("PSPCOM didn't get new sensor data\n");
        // We didn't have new data available; set error flag
    }
    if (xQueueReceive(s_gps_queue, &s_gps_fix, 0) != pdPASS) {
        PAL_LOGW("PSPCOM didn't get new GPS data\n");
        // We didn't have new data available; set error flag
    }
    if (xQueueReceive(s_fp_queue, &s_flight_phase, 0) != pdPASS) {
        PAL_LOGW("PSPCOM didn't get new flight phase data\n");
        // We didn't have new data available; set error flag
    }

    // Standard telemetry
    pspcommsg tx_msg = {
        .payload_len = 19,
        .device_id = PSPCOM_DEVICE_ID,
        .msg_id = STD_TELEM_2,
    };

    // GPS_POS
    gps_pos_packed gps_pos;
    gps_pos.num_sats = s_gps_fix.num_sats & 0x1F;
    gps_pos.lat = ((int32_t)(s_gps_fix.lat / 0.0000108)) & 0x00FFFFFF;
    gps_pos.lon = ((int32_t)(s_gps_fix.lon / 0.0000108)) & 0x01FFFFFF;
    gps_pos.alt = ((uint32_t)(s_gps_fix.height_msl + 1000)) & 0x0003FFFF;
    memcpy(tx_msg.payload, &gps_pos, sizeof(gps_pos_packed));

    // GPS_VEL
    gps_vel_packed gps_vel;
    gps_vel.veln = ((int16_t)(s_gps_fix.vel_north)) & 0x1FFF;
    gps_vel.vele = ((int16_t)(s_gps_fix.vel_east)) & 0x1FFF;
    gps_vel.veld = ((int16_t)(s_gps_fix.vel_down)) & 0x3FFF;
    memcpy(tx_msg.payload + 9, &gps_vel, sizeof(gps_vel_packed));

    // PRES
    uint16_t pres = (uint16_t)(s_sensor_frame.pressure / 0.025);
    tx_msg.payload[14] = pres & 0xFF;
    tx_msg.payload[15] = (pres >> 8) & 0xFF;

    // PYRO_STAT
    uint8_t main_cont = gpio_read(PIN_CONTMAIN);
    uint8_t drg_cont = gpio_read(PIN_CONTDRG);
    uint8_t a1_cont = gpio_read(PIN_CONTA1);
    uint8_t a2_cont = gpio_read(PIN_CONTA2);
    uint8_t a3_cont = gpio_read(PIN_CONTA3);
    // Show continuity and consider all armed
    tx_msg.payload[16] = (main_cont << 1) | (drg_cont << 3) | (a1_cont << 5) |
                         (a2_cont << 7) | 0x55;
    tx_msg.payload[17] = (a3_cont << 1) | 0x1;

    // SYS_STAT
    tx_msg.payload[18] = s_gps_fix.fix_valid & 0x1 & !s_gps_fix.invalid_llh;
    tx_msg.payload[18] |= ((uint8_t)s_flight_phase & 0xF) << 3;

    pspcom_send_msg(tx_msg);
}

static Status start_uart_reading() {
    if (HAL_UART_Receive_DMA(&huart9, cb.buffer, UART_BUFFER_SIZE) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

void task_pspcom_tx() {
    TickType_t last_tx_time = xTaskGetTickCount();

    while (1) {
        // Send a standard telemetry packet
        pspcom_send_standard();

        // Then wait until the next period
        FlightPhase flight_phase = fp_get();
        if (flight_phase < FP_BOOST_1 || flight_phase > FP_DROGUE) {
            // If pre-launch, or on main chute, send less frequently
            vTaskDelayUntil(&last_tx_time,
                            s_config_ptr->pspcom_tx_ground_loop_period_ms /
                                portTICK_PERIOD_MS);
        } else {
            // Otherwise, send more frequently
            vTaskDelayUntil(&last_tx_time,
                            s_config_ptr->pspcom_tx_flight_loop_period_ms /
                                portTICK_PERIOD_MS);
        }
    }
}

/*
static Status stop_uart_reading() {
    if (HAL_UART_AbortReceive(&huart9) != HAL_OK) {
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
        return STATUS_OK;
    }
    vTimerSetTimerID(arm_timer, (void *)pyro);
    if (xTimerStart(arm_timer, 0) != pdPASS) {
        PAL_LOGE("Error starting arm timeout\n");
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
        case 3:
            user_armed[3] = 0;
            break;
        case 4:
            user_armed[4] = 0;
            break;
    }
}

Status pspcom_change_frequency(uint32_t frequency_hz) {
    // Save to the config
    s_config_ptr->telemetry_frequency_hz = frequency_hz;
    config_commit();

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
    pspcom_send_msg(msg);

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

        pspcom_send_msg(msg);
    }

    return STATUS_ERROR;
}

void DMA1_Stream0_IRQHandler(void) {
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    HAL_DMA_IRQHandler(&hdma_uart9_rx);
}

void DMA1_Stream1_IRQHandler(void) {
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);
    HAL_DMA_IRQHandler(&hdma_uart9_tx);
}

void UART9_IRQHandler(void) { HAL_UART_IRQHandler(&huart9); }