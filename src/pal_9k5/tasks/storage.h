#ifndef STORAGE_H
#define STORAGE_H

#include "nand_flash.h"
#include "sd.h"
#include "state_estimation.h"
#include "status.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#define DATA_DIR "/data"

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

void pause_storage();
void start_storage();

Status queue_sensors_for_storage(const SensorFrame* sensor_frame);
Status queue_state_for_storage(const StateFrame* state_frame);
Status queue_gps_for_storage(const GpsFrame* gps_frame);

void task_storage(TaskHandle_t* handle_ptr);

Status storage_write_log(const char* log, size_t size);

#endif  // STORAGE_H
