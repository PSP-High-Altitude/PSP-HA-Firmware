#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <stdint.h>

#include "data.h"
#include "spi/spi.h"
#include "status.h"

Status lsm6dsox_init(SpiDevice* device);  // Initalizing the sensor Who am I,
                                          // set ODR, disable i2c
Accel lsm6dsox_read_accel(
    SpiDevice* device);  // Reading the acceleration registers
Gyro lsm6dsox_read_gyro(SpiDevice* device);  // Reading gyro registers.

#endif // LSM6DSOX_H
