#include "storage.h"

#include <stdio.h>

#include "board.h"
#include "sd.h"
#include "sdmmc/sdmmc.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

/*********************/
/* PERIPHERAL CONFIG */
/*********************/
#ifndef USE_SDMMC
static SdDevice s_sd_conf = {
    .clk = SD_SPEED_10MHz,
    .periph = P_SD4,
};
#else
static SdDevice s_sd_conf = {
    .clk = SD_SPEED_HIGH,
    .periph = P_SD1,
};
#endif  // USE_SDMMC

/********************/
/* STATIC VARIABLES */
/********************/
static QueueHandle_t s_sensor_queue_handle;
static QueueHandle_t s_state_queue_handle;
static QueueHandle_t s_gps_queue_handle;

static bool s_pause_store;

uint64_t g_last_tickless_idle_entry_us;
uint64_t g_total_tickless_idle_us;

/*****************/
/* API FUNCTIONS */
/*****************/
Status init_storage() {
    // For some reason SD init CANNOT go after queue creation
    Status sd_status = EXPECT_OK(sd_init(&s_sd_conf), "SD init");
    if (sd_status != STATUS_OK) {
        return sd_status;
    }

    // Initialize pause flag
    s_pause_store = false;

    // Create the queues
    s_sensor_queue_handle =
        xQueueCreate(SENSOR_QUEUE_LENGTH, SENSOR_QUEUE_ITEM_SIZE);
    s_state_queue_handle =
        xQueueCreate(STATE_QUEUE_LENGTH, STATE_QUEUE_ITEM_SIZE);
    s_gps_queue_handle = xQueueCreate(GPS_QUEUE_LENGTH, GPS_QUEUE_ITEM_SIZE);

    return STATUS_OK;
}

Status queue_sensor_store(SensorFrame* sensor_frame) {
    if (xQueueSend(s_sensor_queue_handle, sensor_frame, 0) != pdPASS) {
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

Status queue_state_store(StateFrame* state_frame) {
    if (xQueueSend(s_state_queue_handle, state_frame, 0) != pdPASS) {
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

Status queue_gps_store(GpsFrame* gps_frame) {
    if (xQueueSend(s_gps_queue_handle, gps_frame, 0) != pdPASS) {
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

void pause_storage() { s_pause_store = true; }

void start_storage() { s_pause_store = false; }

void storage_task() {
    // Initialize LEDs
    gpio_write(PIN_YELLOW, GPIO_LOW);
    gpio_write(PIN_GREEN, GPIO_LOW);

    while (1) {
        // Set disk activity warning LED
        gpio_write(PIN_YELLOW, GPIO_HIGH);

        // Empty the sensor queue
        SensorFrame sensor_frame;
        while (xQueueReceive(s_sensor_queue_handle, &sensor_frame, 1) ==
               pdPASS) {
            if (sd_write_sensor_data(&sensor_frame) != STATUS_OK) {
                break;
            }
        }

        // Empty the state queue
        StateFrame state_frame;
        while (xQueueReceive(s_state_queue_handle, &state_frame, 1) == pdPASS) {
            if (sd_write_state_data(&state_frame) != STATUS_OK) {
                break;
            }
        }

        // Empty the GPS queue
        GpsFrame gps_frame;
        while (xQueueReceive(s_gps_queue_handle, &gps_frame, 1) == pdPASS) {
            if (sd_write_gps_data(&gps_frame) != STATUS_OK) {
                break;
            }
        }

        // Flush everything to SD card
        Status flush_status = EXPECT_OK(sd_flush(), "SD flush");
        gpio_write(PIN_GREEN, flush_status == STATUS_OK);

        // Unset disk activity warning LED
        gpio_write(PIN_YELLOW, GPIO_LOW);

        // Check if the pause flag is set
        if (s_pause_store) {
            // Gather and dump stats
            char prf_buf[1024];  // 40 bytes per task
            printf("Dumping prf stats\n");
            vTaskGetRunTimeStats(prf_buf);
            sd_dump_prf_stats(prf_buf);
            printf(prf_buf);

            // Unmount SD card
            sd_deinit();
            printf("SD safe to remove\n");

            // Record sleep entry time
            g_total_tickless_idle_us = 0;
            uint64_t sleep_entry_us = MICROS();

            // Blink the green LED while waiting
            while (s_pause_store) {
                gpio_write(PIN_GREEN, GPIO_HIGH);
                DELAY(500);
                gpio_write(PIN_GREEN, GPIO_LOW);
                DELAY(500);
            }

            uint64_t sleep_exit_us = MICROS();
            uint64_t tickless_idle_us = g_total_tickless_idle_us;

            // Remount SD card
            printf("Remounting SD\n");
            sd_reinit();

            // Store tickless idle stats
            sprintf(prf_buf, "Tickless idle percentage: %f\n\n",
                    100. * (float)tickless_idle_us /
                        (float)(sleep_exit_us - sleep_entry_us));
            sd_dump_prf_stats(prf_buf);

            // Clear all queues
            xQueueReset(s_sensor_queue_handle);
            xQueueReset(s_state_queue_handle);
            xQueueReset(s_gps_queue_handle);
        }
    }
}
