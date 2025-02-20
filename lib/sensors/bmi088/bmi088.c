#include "bmi088.h"

#include <math.h>
#include <string.h>

#include "timer.h"

static uint8_t s_current_acc_range;
static uint8_t s_current_gyro_range;

static float s_acc_conversion[] = {1.0 / 10920.0, 1.0 / 5460.0, 1.0 / 2730.0,
                                   1 / 1365.0};
static float s_gyro_conversion[] = {1.0 / 16.384, 1.0 / 32.768, 1.0 / 65.536,
                                    1.0 / 131.072, 1.0 / 262.144};

static bool s_initialized = false;

Status bmi088_init(I2cDevice* acc_device, I2cDevice* gyro_device,
                   Bmi088GyroRate gyro_rate, Bmi088AccRate acc_rate,
                   Bmi088GyroRange gyro_range, Bmi088AccRange acc_range) {
    s_current_acc_range = acc_range;
    s_current_gyro_range = gyro_range;
    uint8_t buf[2];

    // Read BMI_ACC & BMI_Gyro register to confirm we're connected
    buf[0] = BMI088_ACC_CHIP_ID;
    if (i2c_write(acc_device, buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (i2c_read(acc_device, buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (buf[0] != 0x1e) {
        return STATUS_ERROR;
    }

    buf[0] = BMI088_GYRO_CHIP_ID;
    if (i2c_write(gyro_device, buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (i2c_read(gyro_device, buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (buf[0] != 0x0f) {
        return STATUS_ERROR;
    }

    // Perform configurations for ACC and activate
    buf[0] = BMI088_ACC_CONF;
    buf[1] = 0xA0 | acc_rate;
    if (i2c_write_verify(acc_device, buf, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }

    buf[0] = BMI088_ACC_RANGE;
    buf[1] = acc_range;
    if (i2c_write_verify(acc_device, buf, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }

    buf[0] = BMI088_ACC_PWR_CONF;
    buf[1] = 0x00;
    if (i2c_write_verify(acc_device, buf, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }

    buf[0] = BMI088_ACC_PWR_CTRL;
    buf[1] = 0x04;
    if (i2c_write_verify(acc_device, buf, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Perform configurations for Gyro
    buf[0] = BMI088_GYRO_RANGE;
    buf[1] = gyro_range;
    if (i2c_write_verify(gyro_device, buf, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }

    buf[0] = BMI088_GYRO_BANDWIDTH;
    buf[1] = 0x80 | gyro_rate;
    if (i2c_write_verify(gyro_device, buf, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }

    s_initialized = true;

    return STATUS_OK;
}

// Read the data from the Acc device
Accel bmi088_acc_read(I2cDevice* device) {
    uint8_t buf[6];
    Accel accel = {NAN, NAN, NAN};

    if (!s_initialized) {
        return accel;
    }

    buf[0] = BMI088_ACC_OUT;

    if (i2c_write(device, buf, 1) != STATUS_OK) {
        return accel;
    }
    if (i2c_read(device, buf, 6) != STATUS_OK) {
        return accel;
    }

    float cf = s_acc_conversion[s_current_acc_range];
    accel.accelX = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]) * cf;
    accel.accelY = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]) * cf;
    accel.accelZ = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]) * cf;

    return accel;
}

// Read data from Gyro device
Gyro bmi088_gyro_read(I2cDevice* device) {
    uint8_t buf[6];
    Gyro gyro = {NAN, NAN, NAN};

    if (!s_initialized) {
        return gyro;
    }

    buf[0] = BMI088_GYRO_OUT;

    if (i2c_write(device, buf, 1) != STATUS_OK) {
        return gyro;
    }
    if (i2c_read(device, buf, 6) != STATUS_OK) {
        return gyro;
    }
    double cf = s_gyro_conversion[s_current_gyro_range];
    gyro.gyroX = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]) * cf;
    gyro.gyroY = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]) * cf;
    gyro.gyroZ = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]) * cf;

    return gyro;
}