#ifndef SD_H
#define SD_H

#include <stdint.h>

#include "gps.pb.h"
#include "sensor.pb.h"
#include "spi/spi.h"
#include "status.h"

// Abstract SD peripheral -- could be SPI or SDMMC underneath
typedef enum {
    P_SD1 = 0,
    P_SD2 = 1,
    P_SD3 = 2,
    P_SD4 = 3,
} SdPeriph;

typedef enum {
    SD_SPEED_INVALID = 0,
    SD_SPEED_100kHz = 100000,
    SD_SPEED_400kHz = 400000,
    SD_SPEED_5MHz = 5000000,
    SD_SPEED_10MHz = 10000000,
    SD_SPEED_25MHz = 25000000,
    SD_SPEED_50MHz = 50000000,
    SD_SPEED_100MHz = 100000000,
} SdSpeed;

typedef struct {
    SdSpeed clk;
    SdPeriph periph;
} SdDevice;

Status diskio_init(SdDevice* device);

Status sd_init(SdDevice* device);

Status sd_reinit();

Status sd_deinit();

Status sd_flush();

Status sd_write_sensor_data(SensorFrame* sensor_frame);

Status sd_write_gps_data(GpsFrame* gps_frame);

#endif  // SD_H
