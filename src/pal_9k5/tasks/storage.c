#include "storage.h"

#include <sys/types.h>

#include "Regex.h"
#include "backup.h"
#include "buttons.h"
#include "fifos.h"
#include "main.h"
#include "pb_create.h"
#include "rtc.h"
#include "stdio.h"
#include "stdlib.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

/********************/
/* STATIC VARIABLES */
/********************/
static BoardConfig* s_config_ptr;

static QueueHandle_t s_sensor_queue;
static QueueHandle_t s_state_queue;
static QueueHandle_t s_gps_queue;

static QueueSetHandle_t s_queue_set;

static char s_logfile_path[64];
extern int g_nand_ready;
static FIL s_logfile;

static uint8_t s_log_buffer[1024];
static FIFO_t s_log_fifo = {
    .buffer = s_log_buffer,
    .size = 1024,
    .circ = 0,
    .head = 0,
    .tail = 0,
    .count = 0,
};

/*****************/
/* API FUNCTIONS */
/*****************/
static void storage_pause_event_handler() {
    // Handle button press
    nand_flash_close_file(&s_logfile);
}

Status init_storage() {
    // Initialize FATFS
    ASSERT_OK(diskio_init(NULL), "diskio init");
    ASSERT_OK(nand_flash_init(), "nand init");

    // Initialize config ptr
    s_config_ptr = get_config_ptr();
    if (s_config_ptr == NULL) {
        ASSERT_OK(STATUS_STATE_ERROR, "unable to get ptr to config\n");
    }

    // Create the queues
    s_sensor_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, SENSOR_QUEUE_ITEM_SIZE);
    s_state_queue = xQueueCreate(STATE_QUEUE_LENGTH, STATE_QUEUE_ITEM_SIZE);
    s_gps_queue = xQueueCreate(GPS_QUEUE_LENGTH, GPS_QUEUE_ITEM_SIZE);
    s_queue_set = xQueueCreateSet(QUEUE_SET_LENGTH);

    // Check that everything was successfully created
    configASSERT(s_sensor_queue);
    configASSERT(s_state_queue);
    configASSERT(s_gps_queue);
    configASSERT(s_queue_set);

    // Add the queues to the set
    xQueueAddToSet(s_sensor_queue, s_queue_set);
    xQueueAddToSet(s_state_queue, s_queue_set);
    xQueueAddToSet(s_gps_queue, s_queue_set);

    // Create the data directory
    if (nand_flash_mkdir(DATA_DIR) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Get a list of files in the data directory
    char** file_list = NULL;
    size_t num_files = 0;
    file_list = nand_flash_get_file_list(DATA_DIR, &num_files);

    // Get the current date for file names
    RTCDateTime dt = rtc_get_datetime();

    int max_num = 0;

    // Find the highest number in the NAND flash
    if (file_list != NULL) {
        // Check if any match the _YYYY-MM-DD-N+. pattern
        Regex regex;
        regexCompile(&regex, "_[0-9]{4}\\-[0-9]{2}\\-[0-9]{2}\\-[0-9]+\\.");

        for (size_t i = 0; i < num_files; i++) {
            Matcher match = regexMatch(&regex, file_list[i]);
            if (match.isFound) {
                // printf("Match: %.*s\n", (int)match.matchLength,
                //        file_list[i] + match.foundAtIndex);

                // Get all the values out of the match
                int year = atoi(file_list[i] + match.foundAtIndex + 1);
                int month = atoi(file_list[i] + match.foundAtIndex + 6);
                int day = atoi(file_list[i] + match.foundAtIndex + 9);
                int num = atoi(file_list[i] + match.foundAtIndex + 12);

                // Skip if the data doesn't match
                if (year != dt.year || month != dt.month || day != dt.day) {
                    continue;
                }

                if (num > max_num) {
                    max_num = num;
                }
            }
        }
    }

    // Free the file list
    nand_flash_delete_file_list(file_list, num_files);

    // Create the file paths
    sprintf(s_logfile_path, DATA_DIR "/log_%04ld-%02ld-%02ld-%d.txt", dt.year,
            dt.month, dt.day, max_num + 1);

    // Open logfile if we are not in MTP mode
    if (!get_backup_ptr()->flag_mtp_pressed) {
        if (nand_flash_open_binary_file(&s_logfile, s_logfile_path) !=
            STATUS_OK) {
            return STATUS_ERROR;
        }
    }

    // Set the pause button handler
    pause_event_callback = storage_pause_event_handler;

    return STATUS_OK;
}

void pause_storage() { s_pause_store = true; }

void start_storage() { s_pause_store = false; }

Status queue_sensors_for_storage(const SensorFrame* sensor_frame) {
    if (xQueueSend(s_sensor_queue, sensor_frame, 0) != pdPASS) {
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

Status queue_state_for_storage(const StateFrame* state_frame) {
    if (xQueueSend(s_state_queue, state_frame, 0) != pdPASS) {
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

Status queue_gps_for_storage(const GpsFrame* gps_frame) {
    if (xQueueSend(s_gps_queue, gps_frame, 0) != pdPASS) {
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

void storage_task() {
    // Initialize LEDs
    // gpio_write(PIN_YELLOW, GPIO_LOW);
    // gpio_write(PIN_GREEN, GPIO_LOW);

    while (1) {
        uint64_t iteration_start_ms = MILLIS();

        while (MILLIS() - iteration_start_ms >
               s_config_ptr->storage_loop_period_ms) {
            // Wait for something to be pushed to a queue
            TickType_t max_wait_ticks =
                pdMS_TO_TICKS(iteration_start_ms +
                              s_config_ptr->storage_loop_period_ms - MILLIS());
            QueueSetMemberHandle_t activated_queue =
                xQueueSelectFromSet(s_queue_set, max_wait_ticks);

            // Receive from the selected queue and store it
            if (activated_queue == s_sensor_queue) {
                SensorFrame sensor_frame;
                xQueueReceive(s_sensor_queue, &sensor_frame, 0);

                size_t sensor_buf_size;
                pb_byte_t* sensor_buf =
                    create_sensor_buffer(&sensor_frame, &sensor_buf_size);
                nand_flash_write_sensor_data(sensor_buf, sensor_buf_size);
            } else if (activated_queue == s_state_queue) {
                StateFrame state_frame;
                xQueueReceive(s_state_queue, &state_frame, 0);

                size_t state_buf_size;
                pb_byte_t* state_buf =
                    create_state_buffer(&state_frame, &state_buf_size);
                nand_flash_write_state_data(state_buf, state_buf_size);
            } else if (activated_queue == s_gps_queue) {
                GpsFrame gps_frame;
                xQueueReceive(s_gps_queue, &gps_frame, 0);

                size_t gps_buf_size;
                pb_byte_t* gps_buf =
                    create_gps_buffer(&gps_frame, &gps_buf_size);
                nand_flash_write_gps_data(gps_buf, gps_buf_size);
            } else {
                // Should print an error but don't want to spam log
            }
        }

        // Set disk activity warning LED
        // gpio_write(PIN_YELLOW, GPIO_HIGH);

        // Flush everything to disk
        Status flush_status = EXPECT_OK(nand_flash_flush(), "NAND flush");
        // gpio_write(PIN_GREEN, flush_status == STATUS_OK);

        // Unset disk activity warning LED
        // gpio_write(PIN_YELLOW, GPIO_LOW);

        // Check if the pause flag is set
        if (s_pause_store) {
            // Gather and dump stats
            char prf_buf[1024];  // 40 bytes per task
            printf("Dumping prf stats\n");
            vTaskGetRunTimeStats(prf_buf);
            sd_dump_prf_stats(prf_buf);
            nand_flash_dump_prf_stats(prf_buf);
            printf(prf_buf);

            // Unmount filesystem
            nand_flash_deinit();
            printf("Unmounted filesystem\n");

            // Blink the green LED while waiting
            while (s_pause_store) {
                // gpio_write(PIN_GREEN, GPIO_HIGH);
                DELAY(500);
                // gpio_write(PIN_GREEN, GPIO_LOW);
                DELAY(500);
            }

            // Remount SD card
            printf("Remounting filesystem\n");
            nand_flash_reinit();
        }
    }
}

Status storage_write_log(const char* log, size_t size) {
    RTCDateTime dt = rtc_get_datetime();

    // Determine size of new buffer
    int new_size =
        snprintf(NULL, 0, "%04ld-%02ld-%02ld %02ld:%02ld:%02ld     %.*s",
                 dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, size,
                 log) +
        1;

    char* new_buf = malloc(new_size);

    snprintf(new_buf, new_size, "%04ld-%02ld-%02ld %02ld:%02ld:%02ld     %.*s",
             dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, size,
             log);

    // Not ready yet, queue the log
    if (!g_nand_ready) {
        fifo_enqueuen(&s_log_fifo, (uint8_t*)new_buf, new_size - 1);

        free(new_buf);
        return STATUS_OK;
    }

    // Write the queued log to the NAND flash
    if (s_log_fifo.count > 0) {
        uint8_t* buf = malloc(s_log_fifo.count);
        int n_read = fifo_dequeuen(&s_log_fifo, buf, s_log_fifo.count);
        Status status =
            nand_flash_write_binary_data(&s_logfile, (uint8_t*)buf, n_read);
        free(buf);

        if (status != STATUS_OK) {
            free(new_buf);
            return status;
        }
    }

    // Write the new log to the NAND flash
    Status status = nand_flash_write_binary_data(&s_logfile, (uint8_t*)new_buf,
                                                 new_size - 1);

    free(new_buf);

    return status;
}
