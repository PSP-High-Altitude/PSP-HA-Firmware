#include "telem.h"

#include "FreeRTOS.h"
#include "board.h"
#include "board_config.h"
#include "pspcom.h"
#include "queue.h"

/********************/
/* STATIC VARIABLES */
/********************/

static QueueHandle_t s_sensor_queue;
static QueueHandle_t s_gps_queue;
static QueueHandle_t s_fp_queue;

static BoardConfig *s_config_ptr = NULL;

/*****************/
/* RADIO DRIVERS */
/*****************/

static Status radio_init();
static Status radio_recv_msg(pspcommsg *msg);
static Status radio_send_msg(pspcommsg *msg);

#ifdef COMPAT_9K5
#include "wlcomm.h"

static Status radio_init() {
    ASSERT_OK(wlcomm_init(), "WL init\n");
    ASSERT_OK(wlcomm_set_freq(s_config_ptr->telemetry_frequency_hz),
              "WL frequency set\n");

    return STATUS_OK;
}

static Status radio_recv_msg(pspcommsg *msg) {
    // Simple wrapper since wlcomm should be self contained
    return wlcomm_recv_msg(msg);
}

static Status radio_send_msg(pspcommsg *msg) {
    // Simple wrapper since wlcomm should be self contained
    return wlcomm_send_msg(msg);
}

#else  // not COMPAT_9K5
#include "sx1276/sx1276.h"

static SpiDevice s_radio_device = {
    .periph = P_SPI2,
    .sck = PIN_PB13,
    .miso = PIN_PC2,
    .mosi = PIN_PC3,
    .cs = PIN_PB12,
    .clk = SPI_SPEED_10MHz,
    .cpha = 0,
    .cpol = 0,
};

static Status radio_init() {
    ASSERT_OK(sx1276_init(&s_radio_device, PIN_PC5,
                          s_config_ptr->telemetry_frequency_hz, 20, 125000, 10,
                          5, 8, false, true, false),
              "LoRa init\n");
    ASSERT_OK(sx1276_start_receive(&s_radio_device), "LoRa recv start\n");

    return STATUS_OK;
}

static Status radio_recv_msg(pspcommsg *msg) {
    int ret = sx1276_packet_available(&s_radio_device);
    if (ret == 1) {
        int len;
        static uint8_t buf[PSPCOM_MAX_PAYLOAD_LEN + 2];
        ret = sx1276_read_packet(&s_radio_device, buf, &len);
        if (ret == STATUS_OK) {
            // Parse message
            msg->device_id = buf[0];
            msg->msg_id = buf[1];
            msg->payload_len = len - 2;
            memcpy(msg->payload, buf + 2, msg->payload_len);
            return STATUS_OK;
        }
    } else if (ret < 0) {
        PAL_LOGE("Error reading sx1276 packet\n");
    }
    return STATUS_ERROR;
}

static Status radio_send_msg(pspcommsg *msg) {
    Status status = STATUS_OK;

    // Prepare packet
    static uint8_t buf[PSPCOM_MAX_PAYLOAD_LEN + 2];
    buf[0] = msg->device_id;
    buf[1] = msg->msg_id;
    memcpy(buf + 2, msg->payload, msg->payload_len);

    // Send message
    UPDATE_STATUS(status,
                  sx1276_transmit(&s_radio_device, buf, 2 + msg->payload_len));

    // When done, start receiving again
    UPDATE_STATUS(status, sx1276_start_receive(&s_radio_device));

    return STATUS_OK;
}
#endif  // not COMPAT_9K5

/*****************/
/* API FUNCTIONS */
/*****************/

Status telem_init() {
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
    return radio_init();
}

void telem_update_sensors(SensorFrame *sensor_frame) {
    xQueueOverwrite(s_sensor_queue, sensor_frame);
}

void telem_update_gps(GPS_Fix_TypeDef *gps_fix) {
    xQueueOverwrite(s_gps_queue, gps_fix);
}

void telem_update_fp(FlightPhase flight_phase) {
    xQueueOverwrite(s_fp_queue, &flight_phase);
}

void task_telem_rx() {
    TickType_t last_rx_time = xTaskGetTickCount();

    while (1) {
        pspcommsg msg;

        if (radio_recv_msg(&msg) == STATUS_OK) {
            pspcom_handle_message(&msg);
        }

        vTaskDelayUntil(&last_rx_time,
                        pdMS_TO_TICKS(s_config_ptr->pspcom_rx_loop_period_ms));
    }
}

void task_telem_tx() {
    TickType_t last_tx_time = xTaskGetTickCount();

    static SensorFrame s_sensor_frame;
    static GPS_Fix_TypeDef s_gps_fix;
    static FlightPhase s_flight_phase;

    while (1) {
        // Get latest data
        if (xQueueReceive(s_sensor_queue, &s_sensor_frame, 0) != pdPASS) {
            PAL_LOGW("telem didn't get new sensor data\n");
            // We didn't have new data available; set error flag
        }
        if (xQueueReceive(s_gps_queue, &s_gps_fix, 0) != pdPASS) {
            PAL_LOGW("telem didn't get new GPS data\n");
            // We didn't have new data available; set error flag
        }
        if (xQueueReceive(s_fp_queue, &s_flight_phase, 0) != pdPASS) {
            PAL_LOGW("telem didn't get new flight phase\n");
            // We didn't have new data available; set error flag
        }

        // Create standard telemetry packet from the received data
        pspcommsg msg =
            pspcom_make_standard(&s_sensor_frame, &s_gps_fix, s_flight_phase);

        // Transmit the packet
        EXPECT_OK(radio_send_msg(&msg), "failed to transmit packet\n");

        // Then wait until the next period
        FlightPhase flight_phase = fp_get();
        if (flight_phase < FP_BOOST || flight_phase > FP_MAIN) {
            // If grounded, send less frequently
            vTaskDelayUntil(
                &last_tx_time,
                pdMS_TO_TICKS(s_config_ptr->pspcom_tx_ground_loop_period_ms));
        } else {
            // Otherwise, send more frequently
            vTaskDelayUntil(
                &last_tx_time,
                pdMS_TO_TICKS(s_config_ptr->pspcom_tx_flight_loop_period_ms));
        }
    }
}
