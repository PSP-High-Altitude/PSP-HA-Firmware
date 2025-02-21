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
#include "storage.h"
#include "string.h"
#include "sx1276/sx1276.h"
#include "task.h"
#include "timer.h"
#include "timers.h"

#define PSPCOM_DEVICE_ID 0x10  // PAL darkstar

#define SENSOR_TELEM_PERIOD 5000
#define GPS_TELEM_PERIOD 5000
#define STATUS_TELEM_PERIOD 5000
#define STD_TELEM_PERIOD_GROUND 5000
#define STD_TELEM_PERIOD_FLIGHT 1000

#define PYRO_TIMEOUT_PERIOD 10000

static uint8_t pyro_armed[5] = {0, 0, 0, 0, 0};
static uint64_t pyro_armed_start[5] = {0, 0, 0, 0, 0};

static QueueHandle_t s_sensor_queue;
static QueueHandle_t s_gps_queue;
static QueueHandle_t s_fp_queue;

static BoardConfig *s_config_ptr = NULL;

SpiDevice s_radio_device = {
    .periph = P_SPI2,
    .sck = PIN_PB13,
    .miso = PIN_PC2,
    .mosi = PIN_PC3,
    .cs = PIN_PB12,
    .clk = SPI_SPEED_10MHz,
    .cpha = 0,
    .cpol = 0,
};

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
    if (s_config_ptr == NULL) {
        return STATUS_ERROR;
    }

    // Queues for synchronization, not buffering
    s_sensor_queue = xQueueCreate(1, sizeof(SensorFrame));
    s_gps_queue = xQueueCreate(1, sizeof(GPS_Fix_TypeDef));
    s_fp_queue = xQueueCreate(1, sizeof(FlightPhase));

    configASSERT(s_sensor_queue);
    configASSERT(s_gps_queue);
    configASSERT(s_fp_queue);

    // Initialize radio
    if (sx1276_init(&s_radio_device, PIN_PC5, 434000000, 20, 125000, 10, 5, 8,
                    false, true, false) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Start receiving
    if (sx1276_start_receive(&s_radio_device) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

void task_pspcom_rx() {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        // Check for new messages
        int ret = sx1276_packet_available(&s_radio_device);
        if (ret == 1) {
            PAL_LOGI("Packet received!\n", ret);
            uint8_t rx_buf[256];
            int len;
            ret = sx1276_read_packet(&s_radio_device, rx_buf, &len);
            if (ret == STATUS_OK) {
                // Parse message
                pspcommsg msg = {
                    .device_id = rx_buf[0],
                    .msg_id = rx_buf[1],
                    .payload_len = len - 2,
                };

                // If the message id is greater than 0x80, the message is
                // telemetry from another board, and we should ignore it
                if (msg.msg_id >= 0x80) {
                    continue;
                }

                memcpy(msg.payload, rx_buf + 2, msg.payload_len);
                printf("Received message: %d %d %d\n", msg.device_id,
                       msg.msg_id, msg.payload_len);

                // Process message
                switch (msg.msg_id) {
                    case ARMMAIN:
                        // Arm main pyro
                        pyro_armed[0] = 1;
                        pyro_armed_start[0] = MILLIS();
                        PAL_LOGI("Main pyro armed!\n");
                        break;
                    case ARMDRG:
                        // Arm drogue pyro
                        pyro_armed[1] = 1;
                        pyro_armed_start[1] = MILLIS();
                        PAL_LOGI("Drogue pyro armed!\n");
                        break;
                    case ARMAUX:
                        if (msg.payload_len == 1 && msg.payload[0] < 3) {
                            // Arm specified aux pyro
                            pyro_armed[msg.payload[0]] = 1;
                            pyro_armed_start[msg.payload[0]] = MILLIS();
                            PAL_LOGI("A%1d pyro armed!\n", msg.payload[0] + 1);
                        } else {
                            // Arm the first aux pyro
                            pyro_armed[2] = 1;
                            pyro_armed_start[2] = MILLIS();
                            PAL_LOGI("A1 pyro armed!\n");
                        }
                        break;
                    case FIREMAIN:
                        // Fire main pyro (if not timed out)
                        if (MILLIS() - pyro_armed_start[0] <
                            PYRO_TIMEOUT_PERIOD) {
                            pyros_fire(PYRO_MAIN);
                        }
                        break;
                    case FIREDRG:
                        // Fire drogue pyro (if not timed out)
                        if (MILLIS() - pyro_armed_start[1] <
                            PYRO_TIMEOUT_PERIOD) {
                            pyros_fire(PYRO_DRG);
                        }
                        break;
                    case FIREAUX:
                        if (msg.payload_len == 1 && msg.payload[0] < 3) {
                            // Fire specified aux pyro (if not timed out)
                            if (MILLIS() - pyro_armed_start[msg.payload[0]] <
                                PYRO_TIMEOUT_PERIOD) {
                                pyros_fire(PYRO_A1 + msg.payload[0]);
                            }
                        } else {
                            // Fire the first aux pyro (if not timed out)
                            if (MILLIS() - pyro_armed_start[2] <
                                PYRO_TIMEOUT_PERIOD) {
                                pyros_fire(PYRO_A1);
                            }
                        }
                        break;
                    case RESETDEVICE:
                        // Reset board if magic byte received
                        if (msg.payload_len == 1 && msg.payload[0] == 0x42) {
                            // Pause storage
                            storage_pause(STORAGE_PAUSE_RESET);
                            while (storage_is_active()) {
                                DELAY_MICROS(1000);
                            }

                            // Reset
                            NVIC_SystemReset();
                        }
                        break;
                }
            }
        } else if (ret < 0) {
            PAL_LOGE("Error reading radio packet!");
        }
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(s_config_ptr->pspcom_rx_loop_period_ms));
    }
}

void pspcom_send_msg(pspcommsg msg) {
    // Prepare packet
    uint8_t msg_buf[2 + msg.payload_len];
    msg_buf[0] = msg.device_id;
    msg_buf[1] = msg.msg_id;
    memcpy(msg_buf + 2, msg.payload, msg.payload_len);

    // Send message
    sx1276_transmit(&s_radio_device, msg_buf, 2 + msg.payload_len);

    // When done, start receiving again
    sx1276_start_receive(&s_radio_device);
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
        // uint8_t main_cont = 0;
        // uint8_t drg_cont = 0;
        // uint8_t a1_cont = 0;
        // uint8_t a2_cont = 0;
        // uint8_t a3_cont = 0;
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
    // uint8_t main_cont = 0;
    // uint8_t drg_cont = 0;
    // uint8_t a1_cont = 0;
    // uint8_t a2_cont = 0;
    // uint8_t a3_cont = 0;
    // Show continuity and consider all armed
    tx_msg.payload[16] = (main_cont << 1) | (drg_cont << 3) | (a1_cont << 5) |
                         (a2_cont << 7) | 0x55;
    tx_msg.payload[17] = (a3_cont << 1) | 0x1;

    // SYS_STAT
    tx_msg.payload[18] = s_gps_fix.fix_valid & 0x1 & !s_gps_fix.invalid_llh;
    tx_msg.payload[18] |= ((uint8_t)s_flight_phase & 0xF) << 3;

    pspcom_send_msg(tx_msg);
}

void task_pspcom_tx() {
    TickType_t last_tx_time = xTaskGetTickCount();

    while (1) {
        // Send a standard telemetry packet
        pspcom_send_standard();

        // Then wait until the next period
        FlightPhase flight_phase = fp_get();
        if (flight_phase < FP_BOOST || flight_phase > FP_DROGUE) {
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

Status pspcom_change_frequency(uint32_t frequency_hz) {
    // Save to the config
    s_config_ptr->telemetry_frequency_hz = frequency_hz;
    config_commit();

    return STATUS_ERROR;
}