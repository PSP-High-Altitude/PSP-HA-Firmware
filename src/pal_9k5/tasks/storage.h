#ifndef STORAGE_H
#define STORAGE_H

#include "nand_flash.h"
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
#define SENSOR_QUEUE_LENGTH (256UL)
#define STATE_QUEUE_LENGTH (64UL)
#define GPS_QUEUE_LENGTH (16UL)
#define QUEUE_SET_LENGTH \
    (SENSOR_QUEUE_LENGTH + STATE_QUEUE_LENGTH + GPS_QUEUE_LENGTH)

// Item sizes
#define SENSOR_QUEUE_ITEM_SIZE (sizeof(SensorFrame))
#define STATE_QUEUE_ITEM_SIZE (sizeof(StateFrame))
#define GPS_QUEUE_ITEM_SIZE (sizeof(GpsFrame))

Status storage_init();

void storage_pause();
void storage_start();

Status storage_queue_sensors(const SensorFrame* sensor_frame);
Status storage_queue_state(const StateFrame* state_frame);
Status storage_queue_gps(const GpsFrame* gps_frame);

void task_storage(TaskHandle_t* handle_ptr);

Status storage_write_log(const char* log, size_t size);

#endif  // STORAGE_H
