#include "i2c/i2c.h"

// Model headers
#include "bmi088/bmi088_model.h"
#include "ms5637/ms5637_model.h"

Status i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len) {
    switch (device->address) {
        case MS5637_I2C_ADDR:
            return ms5637_model_i2c_write(device, tx_buf, len);
            break;
        case BMI088_ACC_I2C_ADDR:
        case BMI088_GYR_I2C_ADDR:
            return bmi088_model_i2c_write(device, tx_buf, len);
            break;
        default:
            // If the address is unknown, the peripheral might not actually
            // raise an error, but we want to detect that something went wrong
            // for the purposes of testing, so raise a testing error
            return STATUS_TESTING_ERROR;
    }
}

Status i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len) {
    switch (device->address) {
        case MS5637_I2C_ADDR:
            return ms5637_model_i2c_read(device, rx_buf, len);
            break;
        case BMI088_ACC_I2C_ADDR:
        case BMI088_GYR_I2C_ADDR:
            return bmi088_model_i2c_read(device, rx_buf, len);
            break;
        default:
            // If the address is unknown, the peripheral might not actually
            // raise an error, but we want to detect that something went wrong
            // for the purposes of testing, so raise a testing error
            return STATUS_TESTING_ERROR;
    }
}
