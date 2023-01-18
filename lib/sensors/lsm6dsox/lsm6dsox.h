#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <stdint.h>

#include "data.h"
#include "spi/spi.h"
#include "status.h"

// Register Definitions
#define LSM6DSOX_CTRL1_XL 0x10
#define LSM6DSOX_CTRL2_G 0x11
#define LSM6DSOX_CTRL3_C 0x12
#define LSM6DSOX_CTRL4_C 0x13
#define LSM6DSOX_CTRL5_C 0x14
#define LSM6DSOX_CTRL6_C 0x15
#define LSM6DSOX_CTRL7_G 0x16
#define LSM6DSOX_CTRL8_XL 0x17
#define LSM6DSOX_CTRL9_XL 0x18
#define LSM6DSOX_CTRL10_C 0x19
#define LSM6DSOX_OUT_TEMP 0x20  // First address of temp registers
#define LSM6DSOX_OUT_G 0x22     // First address of gyro registers
#define LSM6DSOX_OUT_A 0x28     // First address of accel registers

// Settings
typedef enum {
    LSM6DSOX_XL_RATE_12_5_HZ = 0x10,
    LSM6DSOX_XL_RATE_26_HZ = 0x20,
    LSM6DSOX_XL_RATE_52_HZ = 0x30,
    LSM6DSOX_XL_RATE_104_HZ = 0x40,
    LSM6DSOX_XL_RATE_208_HZ = 0x50,
    LSM6DSOX_XL_RATE_416_HZ = 0x60,
    LSM6DSOX_XL_RATE_833_HZ = 0x70,
    LSM6DSOX_XL_RATE_1_66_KHZ = 0x80,
    LSM6DSOX_XL_RATE_3_33_KHZ = 0x90,
    LSM6DSOX_XL_RATE_6_66_KHZ = 0xA0,
} Lsm6dsoxAccelDataRate;

typedef enum {
    LSM6DSOX_XL_RANGE_2_G = 0x00,
    LSM6DSOX_XL_RANGE_4_G = 0x08,
    LSM6DSOX_XL_RANGE_8_G = 0x0C,
    LSM6DSOX_XL_RANGE_16_G = 0x04,
} Lsm6dsoxAccelRange;

typedef enum {
    LSM6DSOX_G_RATE_12_5_HZ = 0x10,
    LSM6DSOX_G_RATE_26_HZ = 0x20,
    LSM6DSOX_G_RATE_52_HZ = 0x30,
    LSM6DSOX_G_RATE_104_HZ = 0x40,
    LSM6DSOX_G_RATE_208_HZ = 0x50,
    LSM6DSOX_G_RATE_416_HZ = 0x60,
    LSM6DSOX_G_RATE_833_HZ = 0x70,
    LSM6DSOX_G_RATE_1_66_KHZ = 0x80,
    LSM6DSOX_G_RATE_3_33_KHZ = 0x90,
    LSM6DSOX_G_RATE_6_66_KHZ = 0xA0,
} Lsm6dsoxGyroDataRate;

typedef enum {
    LSM6DSOX_G_RANGE_125_DPS = 0xFF,
    LSM6DSOX_G_RANGE_250_DPS = 0x00,
    LSM6DSOX_G_RANGE_500_DPS = 0x04,
    LSM6DSOX_G_RANGE_1000_DPS = 0x08,
    LSM6DSOX_G_RANGE_2000_DPS = 0x0C,
} Lsm6dsoxGyroRange;

// Initalize the sensor
Status lsm6dsox_init(SpiDevice* device);

// Read the acceleration registers
Accel lsm6dsox_read_accel(SpiDevice* device);

// Reading gyro registers
Gyro lsm6dsox_read_gyro(SpiDevice* device);

// Set the accelerometer range and measurement rate
Status lsm6dsox_config_accel(SpiDevice* device, Lsm6dsoxAccelDataRate rate,
                             Lsm6dsoxAccelRange range);

// Set the gyroscope range and measurement rate
Status lsm6dsox_config_gyro(SpiDevice* device, Lsm6dsoxGyroDataRate rate,
                            Lsm6dsoxGyroRange range);

#endif // LSM6DSOX_H
