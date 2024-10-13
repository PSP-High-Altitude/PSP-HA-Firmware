#include "storage.h"

#include <sys/types.h>

#include "Regex.h"
#include "backup.h"
#include "buttons.h"
#include "fifos.h"
#include "main.h"
#include "nand_flash.h"
#include "pb_create.h"
#include "rtc/rtc.h"
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

static bool s_pause_store = false;

static char s_logfile_path[64];
static char s_sensfile_path[64];
static char s_statefile_path[64];
static char s_gpsfile_path[64];
extern int g_nand_ready;
static FIL s_logfile;
static FIL s_sensfile;
static FIL s_statefile;
static FIL s_gpsfile;

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
    uint32_t debounce_val = 0x55555555;
    uint64_t start_time = MILLIS();

    // Debounce the button press (with a timeout)
    while (debounce_val != 0x0 && debounce_val != 0xFFFFFFFF) {
        // Timeout
        if (MILLIS() - start_time >= 100) {
            start_storage();  // Default to keeping storage running
            return;
        }

        GpioValue pause_val = gpio_read(PIN_PAUSE);
        if (pause_val == GPIO_ERR) return;

        debounce_val = (debounce_val << 1) | pause_val;
        DELAY_MICROS(100);
    }

    // Decide what to do based on the button transition
    if (debounce_val) {
        pause_storage();
    } else {
        start_storage();
    }
}

static Status storage_open_files() {
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
    sprintf(s_sensfile_path, DATA_DIR "/dat_%04ld-%02ld-%02ld-%d.txt", dt.year,
            dt.month, dt.day, max_num + 1);
    sprintf(s_statefile_path, DATA_DIR "/fsl_%04ld-%02ld-%02ld-%d.txt", dt.year,
            dt.month, dt.day, max_num + 1);
    sprintf(s_gpsfile_path, DATA_DIR "/gps_%04ld-%02ld-%02ld-%d.txt", dt.year,
            dt.month, dt.day, max_num + 1);

    // Open files if we are not in MTP mode
    if (!backup_get_ptr()->flag_mtp_pressed) {
        if (nand_flash_open_binary_file(&s_logfile, s_logfile_path) !=
            STATUS_OK) {
            return STATUS_ERROR;
        }
        if (nand_flash_open_binary_file(&s_sensfile, s_sensfile_path) !=
            STATUS_OK) {
            return STATUS_ERROR;
        }
        if (nand_flash_open_binary_file(&s_statefile, s_statefile_path) !=
            STATUS_OK) {
            return STATUS_ERROR;
        }
        if (nand_flash_open_binary_file(&s_gpsfile, s_gpsfile_path) !=
            STATUS_OK) {
            return STATUS_ERROR;
        }
    }

    return STATUS_OK;
}

Status storage_init() {
    // Initialize FATFS
    ASSERT_OK(diskio_init(NULL), "diskio init");
    ASSERT_OK(nand_flash_init(), "nand init");

    // Initialize config
    ASSERT_OK(config_load(), "load config");

    // Initialize config ptr
    s_config_ptr = config_get_ptr();
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

    // Open files
    if (storage_open_files() != STATUS_OK) {
        return STATUS_ERROR;
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

void task_storage(TaskHandle_t* handle_ptr) {
    // Initialize LEDs
    gpio_write(PIN_YELLOW, GPIO_LOW);
    gpio_write(PIN_GREEN, GPIO_LOW);

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
                nand_flash_write_binary_data(&s_sensfile, sensor_buf,
                                             sensor_buf_size);
            } else if (activated_queue == s_state_queue) {
                StateFrame state_frame;
                xQueueReceive(s_state_queue, &state_frame, 0);

                size_t state_buf_size;
                pb_byte_t* state_buf =
                    create_state_buffer(&state_frame, &state_buf_size);
                nand_flash_write_binary_data(&s_statefile, state_buf,
                                             state_buf_size);
            } else if (activated_queue == s_gps_queue) {
                GpsFrame gps_frame;
                xQueueReceive(s_gps_queue, &gps_frame, 0);

                size_t gps_buf_size;
                pb_byte_t* gps_buf =
                    create_gps_buffer(&gps_frame, &gps_buf_size);
                nand_flash_write_binary_data(&s_gpsfile, gps_buf, gps_buf_size);
            } else {
                // Should print an error but don't want to spam log
            }

            // Set disk activity warning LED
            gpio_write(PIN_YELLOW, GPIO_HIGH);

            // Flush everything to disk
            Status flush_status =
                EXPECT_OK(nand_flash_flush(&s_sensfile), "Sensor flush");
            UPDATE_STATUS(
                flush_status,
                EXPECT_OK(nand_flash_flush(&s_statefile), "State flush"));
            UPDATE_STATUS(flush_status,
                          EXPECT_OK(nand_flash_flush(&s_gpsfile), "GPS flush"));
            UPDATE_STATUS(flush_status,
                          EXPECT_OK(nand_flash_flush(&s_logfile), "Log flush"));
            gpio_write(PIN_GREEN, flush_status == STATUS_OK);

            // Unset disk activity warning LED
            gpio_write(PIN_YELLOW, GPIO_LOW);

            // Check if the pause flag is set
            if (s_pause_store) {
                // Gather and dump stats
                char prf_buf[1024];  // 40 bytes per task
                PAL_LOGI("Dumping prf stats\n");
                vTaskGetRunTimeStats(prf_buf);
                sd_dump_prf_stats(prf_buf);
                nand_flash_dump_prf_stats(prf_buf);
                printf(prf_buf);

                // Unmount filesystem
                nand_flash_deinit();
                PAL_LOGI("Unmounted filesystem\n");

                // Blink the green LED while waiting
                while (s_pause_store) {
                    gpio_write(PIN_GREEN, GPIO_HIGH);
                    DELAY(500);
                    gpio_write(PIN_GREEN, GPIO_LOW);
                    DELAY(500);
                }

                // Remount filesystem
                PAL_LOGI("Remounting filesystem\n");
                nand_flash_reinit();

                // Open new files
                if (storage_open_files() != STATUS_OK) {
                    PAL_LOGE("Error opening new files\n");
                    continue;
                }
            }
        }
    }
}

Status storage_write_log(const char* log, size_t size) {
    // Not ready yet, queue the log
    if (!g_nand_ready) {
        fifo_enqueuen(&s_log_fifo, (uint8_t*)log, size - 1);
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
            return status;
        }
    }

    // Write the new log to the NAND flash
    Status status =
        nand_flash_write_binary_data(&s_logfile, (uint8_t*)log, size - 1);

    return status;
}
