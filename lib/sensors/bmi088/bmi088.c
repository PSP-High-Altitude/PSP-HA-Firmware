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

// Write a register on the BMI088
static Status bmi088_write(I2cDevice* device, uint8_t address, uint8_t* tx_buf,
                           uint8_t len) {
    if (address > 0x7F) {
        return STATUS_PARAMETER_ERROR;
    }

    // Create tx buffer for address
    uint8_t tx_buf_new[len + 1];
    tx_buf_new[0] = address;  // Add address to tx buffer
    memcpy(tx_buf_new + 1, tx_buf, len);

    // Write address and read len bytes
    if (i2c_write(device, tx_buf_new, len + 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Delay between writes (tIDLE_wacc_nm) = 2 us
    DELAY_MICROS(3);

    return STATUS_OK;
}

// Read a register on the BMI088
static Status bmi088_read(I2cDevice* device, uint8_t address, uint8_t* rx_buf,
                          uint8_t len) {
    if (address > 0x7F) {
        return STATUS_PARAMETER_ERROR;
    }

    // Create tx buffer for address
    uint8_t tx_buf[1];
    tx_buf[0] = address;  // Add address to tx buffer

    // Write address and read len bytes
    if (i2c_write(device, tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (i2c_read(device, rx_buf, len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Delay between transmissions (tBUF) = 1.3 us
    DELAY_MICROS(2);

    return STATUS_OK;
}

// Write a register on the BMI088 and verify the data
static Status bmi088_write_verify(I2cDevice* device, uint8_t address,
                                  uint8_t* tx_buf, uint8_t len) {
    if (address > 0x7F) {
        return STATUS_PARAMETER_ERROR;
    }

    // Write the data
    if (bmi088_write(device, address, tx_buf, len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Read back the data
    uint8_t rx_buf[len];
    if (bmi088_read(device, address, rx_buf, len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Verify the data
    if (memcmp(tx_buf, rx_buf, len) != 0) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

Status bmi088_init(I2cDevice* acc_device, I2cDevice* gyro_device,
                   Bmi088GyroRate gyro_rate, Bmi088AccRate acc_rate,
                   Bmi088GyroRange gyro_range, Bmi088AccRange acc_range) {
    s_current_acc_range = acc_range;
    s_current_gyro_range = gyro_range;
    uint8_t buf;

    // Read BMI_ACC & BMI_Gyro register to confirm we're connected
    if (bmi088_read(acc_device, BMI088_ACC_CHIP_ID, &buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (buf != 0x1e) {
        return STATUS_ERROR;
    }

    if (bmi088_read(gyro_device, BMI088_GYRO_CHIP_ID, &buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (buf != 0x0f) {
        return STATUS_ERROR;
    }

    // Perform soft-reset to ensure we are in the POR state
    buf = 0xB6;
    if (bmi088_write(gyro_device, BMI088_ACC_SOFTRESET, &buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Delay at least 1 ms
    DELAY(5);

    buf = 0xB6;
    if (bmi088_write(gyro_device, BMI088_GYRO_SOFTRESET, &buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Delay at least 30 ms
    DELAY(50);

    // Exit accelerometer from PS mode
    buf = 0x04;
    if (bmi088_write_verify(acc_device, BMI088_ACC_PWR_CTRL, &buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Delay 450 us at least
    DELAY(1);

    // Configure accerometer rate
    buf = 0xA0 | acc_rate;
    if (bmi088_write_verify(acc_device, BMI088_ACC_CONF, &buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Configure accelerometer range
    buf = acc_range;
    if (bmi088_write_verify(acc_device, BMI088_ACC_RANGE, &buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Accelometer to active mode
    buf = 0x00;
    if (bmi088_write_verify(acc_device, BMI088_ACC_PWR_CONF, &buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Configure gyroscope range
    buf = gyro_range;
    if (bmi088_write_verify(gyro_device, BMI088_GYRO_RANGE, &buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    buf = 0x80 | gyro_rate;
    if (bmi088_write_verify(gyro_device, BMI088_GYRO_BANDWIDTH, &buf, 1) !=
        STATUS_OK) {
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

    bmi088_read(device, BMI088_ACC_OUT, buf, 6);

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

    bmi088_read(device, BMI088_GYRO_OUT, buf, 6);

    double cf = s_gyro_conversion[s_current_gyro_range];
    gyro.gyroX = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]) * cf;
    gyro.gyroY = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]) * cf;
    gyro.gyroZ = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]) * cf;

    return gyro;
}