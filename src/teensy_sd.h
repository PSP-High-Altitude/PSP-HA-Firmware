#ifndef TEENSY_SD_H
#define TEENSY_SD_H

#include <SD.h>
#include <stdint.h>

#include "data.h"
#include "status.h"

typedef struct {
    uint8_t cs;
} SDDevice;

Status sd_init(SDDevice* device);

Status sd_reinit(SDDevice* device);

Status sd_write(uint64_t timestamp, Accel* accel, Gyro* gyro, BaroData* baro);

#endif  // TEENSY_SD_H
