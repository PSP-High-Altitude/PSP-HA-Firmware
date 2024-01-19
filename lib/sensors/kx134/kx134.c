#include "kx134.h"

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
static Status kx134_read(SpiDevice* device, uint8_t address, uint8_t* rx_buf,
                         uint8_t len) {
    if (address > 0x7F) {
        return STATUS_PARAMETER_ERROR;
    }

    // Create tx buffer with extra dummy bytes
    uint8_t tx_buf[len + 1];
    // Create rx buffer with space for the initial blank byte
    uint8_t rx_buf_new[len + 1];

    tx_buf[0] = (address << 1) | 1;  // Add address and read bit to tx buffer

    // Exchange the address and read len bits then copy all but the first byte
    // received to the original rx_buf.
    Status status = spi_exchange(device, tx_buf, rx_buf_new, len + 1);
    memcpy(rx_buf, rx_buf_new + 1, len);

    DELAY_MICROS(100);

    return status;
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
static Status kx134_write(SpiDevice* device, uint8_t address, uint8_t* tx_buf,
                          uint8_t len) {
    if (address > 0x7F) {
        return STATUS_PARAMETER_ERROR;
    }

    // Create dummy read buffer
    uint8_t rx_buf[len + 1];

    // Create new tx buffer
    uint8_t tx_buf_new[len + 1];
    tx_buf_new[0] = address << 1;  // Add address and write bit to tx buffer
    memcpy(tx_buf_new + 1, tx_buf,
           len);  // Copy bytes to end of the new tx buffer

    // Exchange address and write len bytes
    Status status = spi_exchange(device, tx_buf_new, rx_buf, len + 1);

    DELAY_MICROS(100);

    return status;
}

/**
 * @brief Initialize the KX134 accelerometer
 *
 * @param device SPI device
 * @param rate Output data rate
 * @param range Acceleration range
 * @return Status
 */
Status kx134_init(SpiDevice* device, Kx134OutputDataRate rate,
                  Kx134Range range) {
    uint8_t tx_buf;

    // reset sensor
    tx_buf = 0xBF;
    if (kx134_write(device, KX134_CNTL2, &tx_buf, 1) != STATUS_OK) {
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
Accel kx134_read_accel(SpiDevice* device) {
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
Status kx134_config(SpiDevice* device, Kx134OutputDataRate rate,
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