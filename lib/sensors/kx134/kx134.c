#include "kx134.h"

#include <stdio.h>
#include <string.h>

#include "math.h"
#include "timer.h"

Kx134Range curr_range = 0;

/**
 * @brief Function for SPI read of a KX134 register
 *
 * @param device SPI device
 * @param address register address
 * @param rx_buf buffer to store read data
 * @param len number of bytes to read
 * @return Status
 */
static Status kx134_read(I2cDevice* device, uint8_t address, uint8_t* rx_buf,
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

    // DELAY_MICROS(100);

    return STATUS_OK;
}

/**
 * @brief Function for SPI write of a KX134 register
 *
 * @param device SPI device
 * @param address register address
 * @param tx_buf buffer to write
 * @param len number of bytes to write
 * @return Status
 */
static Status kx134_write(I2cDevice* device, uint8_t address, uint8_t* tx_buf,
                          uint8_t len) {
    if (address > 0x7F) {
        return STATUS_PARAMETER_ERROR;
    }

    // Create tx buffer for address
    uint8_t tx_buf_new[len + 1];
    tx_buf_new[0] = address;              // Add address to tx buffer
    memcpy(tx_buf_new + 1, tx_buf, len);  // Add data to tx buffer

    // Write address and len bytes
    if (i2c_write(device, tx_buf_new, len + 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // DELAY_MICROS(100);

    return STATUS_OK;
}

/**
 * @brief Initialize the KX134 accelerometer
 *
 * @param device SPI device
 * @param rate Output data rate
 * @param range Acceleration range
 * @return Status
 */
Status kx134_init(I2cDevice* device, Kx134OutputDataRate rate,
                  Kx134Range range) {
    uint8_t tx_buf;
    uint8_t rx_buf;

    // power up procedure
    tx_buf = 0;
    if (kx134_write(device, 0x7F, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // power up procedure
    tx_buf = 0x00;
    if (kx134_write(device, KX134_CNTL2, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // reset sensor
    tx_buf = 0x80;
    if (kx134_write(device, KX134_CNTL2, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    DELAY(2);

    // Read WHO_AM_I register
    if (kx134_read(device, KX134_WHO_AM_I, &rx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (rx_buf != 0x46) {
        return STATUS_ERROR;
    }

    // Read COTR register
    if (kx134_read(device, KX134_COTR, &rx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (rx_buf != 0x55) {
        return STATUS_ERROR;
    }

    // Configure the accel to 25600hz rate
    tx_buf = rate;
    if (kx134_write(device, KX134_ODCNTL, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // enable sensor and set range
    curr_range = range;
    tx_buf = 0xC0 | range;
    if (kx134_write(device, KX134_CNTL1, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/**
 * @brief Read the KX134 accelerometer
 *
 * @param device SPI device
 * @return Accel struct
 */
Accel kx134_read_accel(I2cDevice* device) {
    Accel result;

    // Read all 6 registers at once
    uint8_t rx_buf[6];
    if (kx134_read(device, KX134_DATA, rx_buf, 6) != STATUS_OK) {
        result.accelX = NAN;
        result.accelY = NAN;
        result.accelZ = NAN;
        return result;
    }

    // Convert unsigned 8-bit halves to signed 16-bit numbers
    int16_t acc_x_raw = (int16_t)(((uint16_t)rx_buf[1] << 8) | rx_buf[0]);
    int16_t acc_y_raw = (int16_t)(((uint16_t)rx_buf[3] << 8) | rx_buf[2]);
    int16_t acc_z_raw = (int16_t)(((uint16_t)rx_buf[5] << 8) | rx_buf[4]);

    float conversion_factor;
    switch (curr_range) {
        case KX134_RANGE_8_G:
            conversion_factor = 1.0 / 4096;
            break;
        case KX134_RANGE_16_G:
            conversion_factor = 1.0 / 2048;
            break;
        case KX134_RANGE_32_G:
            conversion_factor = 1.0 / 1024;
            break;
        case KX134_RANGE_64_G:
            conversion_factor = 1.0 / 512;
            break;
        default:
            conversion_factor = 1.0 / 512;
            break;
    }

    float acc_x = (float)acc_x_raw * conversion_factor;
    float acc_y = (float)acc_y_raw * conversion_factor;
    float acc_z = (float)acc_z_raw * conversion_factor;

    result.accelX = acc_x;
    result.accelY = acc_y;
    result.accelZ = acc_z;

    return result;
}

/**
 * @brief Configure the KX134 accelerometer
 *
 * @param device SPI device
 * @param rate Output data rate
 * @param range Acceleration range
 * @return Status
 */
Status kx134_config(I2cDevice* device, Kx134OutputDataRate rate,
                    Kx134Range range) {
    uint8_t tx_buf;

    // disable sensor
    tx_buf = 0x0;
    if (kx134_write(device, KX134_CNTL1, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Configure the accel to 25600hz rate
    tx_buf = rate;
    if (kx134_write(device, KX134_ODCNTL, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // enable sensor and set range
    tx_buf = 0xC0 | range;
    if (kx134_write(device, KX134_CNTL1, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    uint8_t rx_buf;

    // Verify the settings
    Status status = kx134_read(device, KX134_ODCNTL, &rx_buf, 1);

    if (status != STATUS_OK) {
        return status;
    }

    if ((rx_buf & 0xF) != rate) {
        return STATUS_ERROR;
    }

    status = kx134_read(device, KX134_CNTL1, &rx_buf, 1);

    if (status != STATUS_OK) {
        return status;
    }

    if ((rx_buf & 0x18) != rate) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}