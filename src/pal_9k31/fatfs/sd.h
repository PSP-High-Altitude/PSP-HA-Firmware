#ifndef SD_H
#define SD_H

#include <stdint.h>

#include "data.h"
#include "spi/spi.h"
#include "status.h"

Status sd_spi_init(SpiDevice* device);

Status sd_init(SpiDevice* device);

Status sd_reinit(SpiDevice* device);

Status sd_write(uint64_t timestamp, Accel* accel, Gyro* gyro, BaroData* baro);

#endif  // SD_H
