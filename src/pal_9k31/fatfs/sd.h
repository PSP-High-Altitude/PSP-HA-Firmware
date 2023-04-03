#ifndef SD_H
#define SD_H

#include <stdint.h>

#include "data.h"
#include "max_m10s.h"
#include "spi/spi.h"
#include "status.h"

Status sd_spi_init(SpiDevice* device);

Status sd_init(SpiDevice* device);

Status sd_reinit();

Status sd_deinit();

Status sd_write(uint64_t timestamp, Accel* accel, Gyro* gyro, BaroData* baro,
                Mag* mag, GPS_Fix_TypeDef* fix);

#endif  // SD_H
