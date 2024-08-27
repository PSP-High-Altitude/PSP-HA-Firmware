#ifndef BMI088_MODEL_H
#define BMI088_MODEL_H

#include <stdint.h>

#include "i2c/i2c.h"
#include "status.h"

#define BMI088_ACC_I2C_ADDR (uint8_t)0x18
#define BMI088_GYR_I2C_ADDR (uint8_t)0x68

#define BMI088_ACC_CHIP_ID 0x00
#define BMI088_ACC_DATA 0x12
#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_RANGE 0x41

#define BMI088_GYR_CHIP_ID 0x00
#define BMI088_GYR_DATA 0x02
#define BMI088_GYR_RANGE 0x0F
#define BMI088_GYR_BANDWIDTH 0x10

Status bmi088_model_i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len);

Status bmi088_model_i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len);

#endif  // BMI088_MODEL_H
