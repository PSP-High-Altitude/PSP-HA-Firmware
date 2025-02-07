#include "storage.h"

#include <sys/types.h>

#include "Regex.h"
#include "backup/backup.h"
#include "buttons.h"
#include "fatlog.h"
#include "fifos.h"
#include "main.h"
#include "pb_create.h"
#include "rtc/rtc.h"
#include "sdmmc/sdmmc.h"
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

static uint32_t s_sensor_overflows;
static uint32_t s_state_overflows;
static uint32_t s_gps_overflows;

static QueueHandle_t s_sensor_queue;
static QueueHandle_t s_state_queue;
static QueueHandle_t s_gps_queue;

static QueueSetHandle_t s_queue_set;

static bool s_storage_active = false;
static uint32_t s_pause_mode = 0;

static char s_logfile_path[64];
static char s_datfile_path[64];
static char s_fslfile_path[64];
static char s_gpsfile_path[64];

static FIL s_logfile;
static FIL s_datfile;
static FIL s_fslfile;
static FIL s_gpsfile;

static uint8_t s_header[] = FIRMWARE_SPECIFIER "\n";

static uint8_t s_log_buffer[4096];
static FIFO_t s_log_fifo = {
    .buffer = s_log_buffer,
    .size = 4096,
    .circ = 0,
    .head = 0,
    .tail = 0,
    .count = 0,
};

static SdmmcDevice s_sdmmc_device = {
    .clk = SD_SPEED_DEFAULT,
    .periph = P_SD2,
};

static char s_prf_buf[1024];  // 40 bytes per task

/*****************/
/* API FUNCTIONS */
/*****************/
static void storage_save_event_handler() {
    // Handle button press
    uint32_t debounce_val = 0x55555555;
    uint64_t start_time = MILLIS();

    // Debounce the button press (with a timeout)
    while (debounce_val != 0x0 && debounce_val != 0xFFFFFFFF) {
        // Timeout
        if (MILLIS() - start_time >= 100) {
            // Default to keeping storage running
            storage_start(STORAGE_PAUSE_SAFE);
            return;
        }

        GpioValue pause_val = gpio_read(PIN_PAUSE);
        if (pause_val == GPIO_ERR) return;

        debounce_val = (debounce_val << 1) | pause_val;
        DELAY_MICROS(100);
    }

    // Decide what to do based on the button transition
    if (debounce_val) {
        storage_pause(STORAGE_PAUSE_SAFE);
    } else {
        storage_start(STORAGE_PAUSE_SAFE);
    }
}

static Status storage_close_files() {
    ASSERT_OK(fatlog_close_file(&s_logfile), "failed to close log\n");
    ASSERT_OK(fatlog_close_file(&s_datfile), "failed to close sens\n");
    ASSERT_OK(fatlog_close_file(&s_fslfile), "failed to close state\n");
    ASSERT_OK(fatlog_close_file(&s_gpsfile), "failed to close gps\n");

    return STATUS_OK;
}

static Status storage_open_files() {
    // Get a list of files in the data directory
    char** file_list = NULL;
    size_t num_files = 0;
    file_list = fatlog_get_file_list(LOG_DIR, &num_files);

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
    fatlog_delete_file_list(file_list, num_files);

    // Create the file paths
    sprintf(s_logfile_path, LOG_DIR "/log_%04ld-%02ld-%02ld-%d.txt", dt.year,
            dt.month, dt.day, max_num + 1);
    sprintf(s_datfile_path, SENSOR_DIR "/dat_%04ld-%02ld-%02ld-%d.pb3", dt.year,
            dt.month, dt.day, max_num + 1);
    sprintf(s_fslfile_path, STATE_DIR "/fsl_%04ld-%02ld-%02ld-%d.pb3", dt.year,
            dt.month, dt.day, max_num + 1);
    sprintf(s_gpsfile_path, GPS_DIR "/gps_%04ld-%02ld-%02ld-%d.pb3", dt.year,
            dt.month, dt.day, max_num + 1);

    // Open files if we are not in MTP mode
    if (!backup_get_ptr()->flag_mtp_pressed) {
        ASSERT_OK(fatlog_open_file_for_write(&s_logfile, s_logfile_path),
                  "failed to open log\n");
        ASSERT_OK(fatlog_open_file_for_write(&s_datfile, s_datfile_path),
                  "failed to open sensor\n");
        ASSERT_OK(fatlog_open_file_for_write(&s_fslfile, s_fslfile_path),
                  "failed to open state\n");
        ASSERT_OK(fatlog_open_file_for_write(&s_gpsfile, s_gpsfile_path),
                  "failed to open gps\n");

        // Write the header to each file
        ASSERT_OK(
            fatlog_write_data(&s_logfile, s_header, strlen((char*)s_header)),
            "failed to write log header\n");
        ASSERT_OK(
            fatlog_write_data(&s_datfile, s_header, strlen((char*)s_header)),
            "failed to write sensor header\n");
        ASSERT_OK(
            fatlog_write_data(&s_fslfile, s_header, strlen((char*)s_header)),
            "failed to write state header\n");
        ASSERT_OK(
            fatlog_write_data(&s_gpsfile, s_header, strlen((char*)s_header)),
            "failed to write gps header\n");
    }

    return STATUS_OK;
}

Status storage_init() {
    // Initialize FATFS
    ASSERT_OK(diskio_init(&s_sdmmc_device), "diskio init");
    ASSERT_OK(fatlog_init(), "fatlog init");

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
    ASSERT_OK(fatlog_mkdir(LOG_DIR), "failed to create log dir\n");
    ASSERT_OK(fatlog_mkdir(SENSOR_DIR), "failed to create sensor dir\n");
    ASSERT_OK(fatlog_mkdir(STATE_DIR), "failed to create state dir\n");
    ASSERT_OK(fatlog_mkdir(GPS_DIR), "failed to create gps dir\n");

    // Open files
    if (storage_open_files() != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Set the pause button handler
    pause_event_callback = storage_save_event_handler;

    return STATUS_OK;
}

bool storage_is_active() { return s_storage_active; }

void storage_pause(StoragePauseMode mode) { s_pause_mode |= mode; }

void storage_start(StoragePauseMode mode) { s_pause_mode &= ~mode; }

Status storage_queue_sensors(const SensorFrame* sensor_frame) {
    if (xQueueSend(s_sensor_queue, sensor_frame, 0) != pdPASS) {
        s_sensor_overflows += 1;
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

Status storage_queue_state(const StateFrame* state_frame) {
    if (xQueueSend(s_state_queue, state_frame, 0) != pdPASS) {
        s_state_overflows += 1;
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

Status storage_queue_gps(const GpsFrame* gps_frame) {
    if (xQueueSend(s_gps_queue, gps_frame, 0) != pdPASS) {
        s_gps_overflows += 1;
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

void task_storage(TaskHandle_t* handle_ptr) {
    Status status = STATUS_OK;

    // Outer loop does initialization and teardown
    while (1) {
        // Initialize LEDs
        gpio_write(PIN_YELLOW, GPIO_LOW);
        gpio_write(PIN_GREEN, GPIO_LOW);

        s_storage_active = true;
        PAL_LOGI("Starting FS reinit\n");
        gpio_write(PIN_YELLOW, GPIO_HIGH);

        // Make sure filesystem is mounted
        UPDATE_STATUS(status, EXPECT_OK(fatlog_reinit(), "FS reinit\n"));

        // Open files
        UPDATE_STATUS(status, EXPECT_OK(storage_open_files(), "File open\n"));

        PAL_LOGI("Entering main storage loop\n");

        while (status == STATUS_OK && !s_pause_mode) {
            uint64_t iteration_start_ms = MILLIS();
            uint32_t stored_items = 0;

            while (MILLIS() - iteration_start_ms <
                   s_config_ptr->storage_loop_period_ms) {
                // Wait for something to be pushed to a queue
                TickType_t max_wait_ticks = pdMS_TO_TICKS(
                    iteration_start_ms + s_config_ptr->storage_loop_period_ms -
                    MILLIS());
                QueueSetMemberHandle_t activated_queue =
                    xQueueSelectFromSet(s_queue_set, max_wait_ticks);

                // Receive from the selected queue and store it
                if (activated_queue == s_sensor_queue) {
                    SensorFrame sensor_frame;
                    xQueueReceive(s_sensor_queue, &sensor_frame, 0);

                    size_t sensor_buf_size;
                    pb_byte_t* sensor_buf =
                        create_sensor_buffer(&sensor_frame, &sensor_buf_size);
                    fatlog_write_data(&s_datfile, sensor_buf, sensor_buf_size);
                } else if (activated_queue == s_state_queue) {
                    StateFrame state_frame;
                    xQueueReceive(s_state_queue, &state_frame, 0);

                    size_t state_buf_size;
                    pb_byte_t* state_buf =
                        create_state_buffer(&state_frame, &state_buf_size);
                    fatlog_write_data(&s_fslfile, state_buf, state_buf_size);
                } else if (activated_queue == s_gps_queue) {
                    GpsFrame gps_frame;
                    xQueueReceive(s_gps_queue, &gps_frame, 0);

                    size_t gps_buf_size;
                    pb_byte_t* gps_buf =
                        create_gps_buffer(&gps_frame, &gps_buf_size);
                    fatlog_write_data(&s_gpsfile, gps_buf, gps_buf_size);
                } else {
                    // Should print an error but don't want to spam log
                    // and anyway this should never happen in the first place
                }

                stored_items += 1;
            }

            // Flush everything to disk
            UPDATE_STATUS(status,
                          EXPECT_OK(fatlog_flush(&s_datfile), "Sensor flush"));
            UPDATE_STATUS(status,
                          EXPECT_OK(fatlog_flush(&s_fslfile), "State flush"));
            UPDATE_STATUS(status,
                          EXPECT_OK(fatlog_flush(&s_gpsfile), "GPS flush"));
            UPDATE_STATUS(status,
                          EXPECT_OK(fatlog_flush(&s_logfile), "Log flush"));

            gpio_write(PIN_GREEN, status == STATUS_OK);
            gpio_write(PIN_YELLOW, GPIO_LOW);

            // Check for and log any queue overflows
            if (s_sensor_overflows) {
                PAL_LOGW("%lu overflows in sensor queue\n", s_sensor_overflows);
                s_sensor_overflows = 0;
            }
            if (s_state_overflows) {
                PAL_LOGW("%lu overflows in state queue\n", s_state_overflows);
                s_state_overflows = 0;
            }
            if (s_gps_overflows) {
                PAL_LOGW("%lu overflows in gps queue\n", s_gps_overflows);
                s_gps_overflows = 0;
            }
        }

        PAL_LOGI("Exited main storage loop\n");

        if (s_pause_mode) {
            // Gather and dump stats when pausing
            vTaskGetRunTimeStats(s_prf_buf);
            PAL_LOGI("Profiling stats:\n%s\n", s_prf_buf);
        }

        // Unmount filesystem
        EXPECT_OK(storage_close_files(), "File close\n");
        EXPECT_OK(fatlog_deinit(), "FS unmount\n");

        gpio_write(PIN_YELLOW, GPIO_LOW);
        PAL_LOGI("Finished FS deinit\n");
        s_storage_active = false;

        // Blink the green LED while waiting
        while (s_pause_mode) {
            gpio_write(PIN_GREEN, status == STATUS_OK);
            DELAY(250);
            gpio_write(PIN_GREEN, GPIO_LOW);
            DELAY(250);
        }

        // Allow some time before restarting
        DELAY(500);

        // Reset status since restarting
        status = STATUS_OK;
    }
}

Status storage_write_log(const char* log, size_t size) {
    // Write the queued log to the NAND flash
    if (s_log_fifo.count > 0) {
        uint8_t* buf = malloc(s_log_fifo.count);

        int n_read = fifo_dequeuen(&s_log_fifo, buf, s_log_fifo.count);
        Status status = fatlog_write_data(&s_logfile, buf, n_read);

        if (status != STATUS_OK) {
            // If it fails, put the dequeued data back into the FIFO
            fifo_enqueuen(&s_log_fifo, buf, n_read);
        }

        free(buf);
    }

    // Write the new log to the NAND flash
    Status status = fatlog_write_data(&s_logfile, (uint8_t*)log, size);

    if (status != STATUS_OK) {
        // If it fails, add the new data to the log
        fifo_enqueuen(&s_log_fifo, (uint8_t*)log, size);
    }

    return status;
}
