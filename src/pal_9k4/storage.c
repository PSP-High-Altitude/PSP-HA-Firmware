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
static QueueHandle_t s_sensor_queue_handle = NULL;
static QueueHandle_t s_gps_queue_handle = NULL;

static bool s_pause_store = false;

uint64_t g_last_tickless_idle_entry_us;
uint64_t g_total_tickless_idle_us;

/*****************/
/* API FUNCTIONS */
/*****************/
Status init_storage() {
    // For some reason SD init CANNOT go after queue creation
    Status sd_status = EXPECT_OK(sd_init(&s_sd_conf), "SD init");

    s_sensor_queue_handle =
        xQueueCreate(SENSOR_QUEUE_LENGTH, SENSOR_QUEUE_ITEM_SIZE);
    s_gps_queue_handle = xQueueCreate(GPS_QUEUE_LENGTH, GPS_QUEUE_ITEM_SIZE);

    return sd_status;
}

Status queue_sensor_store(SensorFrame* sensor_frame) {
    if (!s_sensor_queue_handle) {
        return STATUS_ERROR;
    }

    if (xQueueSend(s_sensor_queue_handle, sensor_frame, 0) != pdPASS) {
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

Status queue_gps_store(GpsFrame* gps_frame) {
    if (!s_gps_queue_handle) {
        return STATUS_ERROR;
    }

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
        bool write_successful = false;
        SensorFrame sensor_frame;
        GpsFrame gps_frame;

        if (xQueueReceive(s_sensor_queue_handle, &sensor_frame, 1) != pdPASS) {
            memset(&sensor_frame, 0, sizeof(sensor_frame));
        }
        if (xQueueReceive(s_gps_queue_handle, &gps_frame, 10000) == pdPASS) {
            sd_write_data(&sensor_frame, &gps_frame);
            write_successful = true;
        }

        // Flush everything to SD card
        Status flush_status = EXPECT_OK(sd_flush(), "SD flush");
        gpio_write(PIN_GREEN, flush_status == STATUS_OK && write_successful);

        // Unset disk activity warning LED
        gpio_write(PIN_YELLOW, GPIO_LOW);

        printf("GPS Frame:\n");
        printf("  Timestamp: %lu\n", (uint32_t)gps_frame.timestamp);
        printf("  UTC Time: %04lu-%02lu-%02lu %02lu:%02lu:%02lu\n",
               gps_frame.year, gps_frame.month, gps_frame.day, gps_frame.hour,
               gps_frame.min, gps_frame.sec);
        printf("  Number of Satellites: %lu\n", gps_frame.num_sats);
        printf("  Longitude: %.6f\n", gps_frame.lon);
        printf("  Latitude: %.6f\n", gps_frame.lat);
        printf("  Height: %.2f m\n", gps_frame.height);
        printf("  Height MSL: %.2f m\n", gps_frame.height_msl);
        printf("  Horizontal Accuracy: %.2f m\n", gps_frame.accuracy_horiz);
        printf("  Vertical Accuracy: %.2f m\n", gps_frame.accuracy_vertical);
        printf("  Velocity (North): %.2f m/s\n", gps_frame.vel_north);
        printf("  Velocity (East): %.2f m/s\n", gps_frame.vel_east);
        printf("  Velocity (Down): %.2f m/s\n", gps_frame.vel_down);
        printf("  Ground Speed: %.2f m/s\n", gps_frame.ground_speed);
        printf("  Heading: %.2f degrees\n", gps_frame.hdg);
        printf("\n");

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
        }
    }
}
