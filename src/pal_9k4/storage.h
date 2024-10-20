#ifndef STORAGE_H
#define STORAGE_H

#include "gps.pb.h"
#include "sensor.pb.h"
#include "state.pb.h"
#include "status.h"

// Flag to indicate whether to use SDMMC or SPI peripheral
#define USE_SDMMC

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

Status storage_init(uint8_t usb_mode);

void storage_pause();
void storage_start();

void storage_task();

Status queue_sensor_store(SensorFrame* sensor_frame);
Status queue_state_store(StateFrame* state_frame);
Status queue_gps_store(GpsFrame* gps_frame);

#endif  // STORAGE_H
