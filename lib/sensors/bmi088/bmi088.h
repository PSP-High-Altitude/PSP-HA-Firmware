#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>

#include "data.h"
#include "i2c/i2c.h"
#include "status.h"

// Register Definitions
// Accelerometer defs
#define BMI088_ACC_CHIP_ID 0x00
#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACC_STATUS 0x03
#define BMI088_ACC_X_LSB 0x12
#define BMI088_ACC_X_MSB 0x13
#define BMI088_ACC_Y_LSB 0x14
#define BMI088_ACC_Y_MSB 0x15
#define BMI088_ACC_Z_LSB 0x16
#define BMI088_ACC_Z_MSB 0x17
#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_RANGE 0x41
#define BMI088_ACC_OUT BMI088_ACC_X_LSB
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_CTRL 0x7D

// Gyroscope defs
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_X_LSB 0x02
#define BMI088_GYRO_X_MSB 0x03
#define BMI088_GYRO_Y_LSB 0x04
#define BMI088_GYRO_Y_MSB 0x05
#define BMI088_GYRO_Z_LSB 0x06
#define BMI088_GYRO_Z_MSB 0x07
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_BANDWIDTH 0x10
#define BMI088_GYRO_OUT BMI088_GYRO_X_LSB

// Settings
typedef enum {
    BMI088_ACC_RANGE_3_G = 0x00,
    BMI088_ACC_RANGE_6_G = 0x01,
    BMI088_ACC_RANGE_12_G = 0x02,
    BMI088_ACC_RANGE_24_G = 0x03
} Bmi088AccRange;

typedef enum {
    BMI088_ACC_RATE_12_5_HZ = 0x05,
    BMI088_ACC_RATE_25_HZ = 0x06,
    BMI088_ACC_RATE_50_HZ = 0x07,
    BMI088_ACC_RATE_100_HZ = 0x08,
    BMI088_ACC_RATE_200_HZ = 0x09,
    BMI088_ACC_RATE_400_HZ = 0x0A,
    BMI088_ACC_RATE_800_HZ = 0x0B,
    BMI088_ACC_RATE_1600_HZ = 0x0C
} Bmi088AccRate;

typedef enum {
    BMI088_GYRO_RANGE_2000_DPS = 0x00,
    BMI088_GYRO_RANGE_1000_DPS = 0x01,
    BMI088_GYRO_RANGE_500_DPS = 0x02,
    BMI088_GYRO_RANGE_250_DPS = 0x03,
    BMI088_GYRO_RANGE_125_DPS = 0x04
} Bmi088GyroRange;

typedef enum {
    BMI088_GYRO_RATE_2000_HZ = 0x00,
    BMI088_GYRO_RATE_1000_HZ = 0x02,
    BMI088_GYRO_RATE_400_HZ = 0x03,
    BMI088_GYRO_RATE_200_HZ = 0x04,
    BMI088_GYRO_RATE_100_HZ = 0x05
} Bmi088GyroRate;

typedef enum {
    BMI088_GYRO_BW_2000_HZ = 0x01,
    BMI088_GYRO_BW_1000_HZ = 0x02,
    BMI088_GYRO_BW_400_HZ = 0x03,
    BMI088_GYRO_BW_200_HZ = 0x04,
    BMI088_GYRO_BW_100_HZ = 0x05
} Bmi088GyroBandwidth;

// Initalize the sensor
Status bmi088_init(I2cDevice* acc_device, I2cDevice* gyro_device,
                   Bmi088GyroRate gyro_rate, Bmi088AccRate acc_rate,
                   Bmi088GyroRange gyro_range, Bmi088AccRange acc_range);

// Read the acceleration registers
Accel bmi088_acc_read(I2cDevice* device);
Gyro bmi088_gyro_read(I2cDevice* device);

#endif  // BMI088_H

// God I hope this is right