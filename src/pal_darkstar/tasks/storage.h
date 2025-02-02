#ifndef STORAGE_H
#define STORAGE_H

#include "sd.h"
#include "state_estimation.h"
#include "status.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#define LOG_DIR "/log"
#define SENSOR_DIR "/sensor"
#define STATE_DIR "/state"
#define GPS_DIR "/gps"

// Queue lengths
#define SENSOR_QUEUE_LENGTH (64UL)
#define STATE_QUEUE_LENGTH (64UL)
#define GPS_QUEUE_LENGTH (16UL)
#define QUEUE_SET_LENGTH \
    (SENSOR_QUEUE_LENGTH + STATE_QUEUE_LENGTH + GPS_QUEUE_LENGTH)

// Item sizes
#define SENSOR_QUEUE_ITEM_SIZE (sizeof(SensorFrame))
#define STATE_QUEUE_ITEM_SIZE (sizeof(StateFrame))
#define GPS_QUEUE_ITEM_SIZE (sizeof(GpsFrame))

// Enum for distinguishing pause reasons
// NOTE: all enum members must have a value
// representing a distinct bit pos (1, 2, 4, 8, ...)
typedef enum {
    STORAGE_PAUSE_RESET = 1,
    STORAGE_PAUSE_SAFE = 2,
    STORAGE_PAUSE_MSC = 4,
} StoragePauseMode;

Status storage_init();

bool storage_is_active();

void storage_pause(StoragePauseMode mode);
void storage_start(StoragePauseMode mode);

Status storage_queue_sensors(const SensorFrame* sensor_frame);
Status storage_queue_state(const StateFrame* state_frame);
Status storage_queue_gps(const GpsFrame* gps_frame);

void task_storage(TaskHandle_t* handle_ptr);

Status storage_write_log(const char* log, size_t size);

#endif  // STORAGE_H
