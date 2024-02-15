#ifndef SENSORS_H
#define SENSORS_H

#include "sensor.pb.h"
#include "status.h"

// Flag to indicate whether to use SDMMC or SPI peripheral
#define USE_SDMMC

// Queue lengths
#define SENSOR_QUEUE_LENGTH (256)
#define STATE_QUEUE_LENGTH (64)
#define GPS_QUEUE_LENGTH (8)
#define QUEUE_SET_LENGTH \
    (SENSOR_QUEUE_LENGTH + STATE_QUEUE_LENGTH + GPS_QUEUE_LENGTH)

// Item sizes
#define SENSOR_QUEUE_ITEM_SIZE (sizeof(SensorFrame))
#define STATE_QUEUE_ITEM_SIZE (sizeof(StateFrame))
#define GPS_QUEUE_ITEM_SIZE (sizeof(GpsFrame))

Status init_sensors(uint32_t polling_period_ms);

void pause_storage();
void start_storage();

void read_sensors_task();

#endif  // SENSORS_H
