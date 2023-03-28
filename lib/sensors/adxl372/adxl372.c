#include "adxl372.h"

#include <string.h>

#include "math.h"
#include "timer.h"

/*
 * Questions:
 * No extrenal Clock right?
 * Do we use multithreading? probably not right?
 * Should I use lower bandwidth to decrease noise? What are the current
 * restrictions on this device? (page 16 Power/noise tradeoff) Do we need
 * Low-Pass or High-Pass Antializing Filter? (page 15-16 Bandwidth) What is
 * uint8_t and where is it defined? How can I just save a boolean? Conversion
 * factors?
 */

/*
 * TODO:
 * 1) implement autonomous event detection?
 */

/*
 * ~Prepocessing required~ Make sure adxl372 is not being written to when
 * reading Additionally, ensure FIFO mode is deactivated. Bypass FIFO mode by
 * setting indicies [2:1] (zero set) to 0 at address 0x3A. Instruction on
 * disabling FIFO are found on page 23 of ADXL data sheet.
 */
static Status adxl372_read(SpiDevice* device, uint8_t address, uint8_t* rx_buf,
                           uint8_t len) {
    if (address > 127) {
        return STATUS_PARAMETER_ERROR;
    }

    // Create tx buffer with extra dummy bytes
    uint8_t tx_buf[len + 1];
    // Create rx buffer with space for the initial blank byte
    uint8_t rx_buf_new[len + 1];

    tx_buf[0] = address | 0x80;  // Add address and read bit to tx buffer

    // Exchange the address and read len bits then copy all but the first byte
    // received to the original rx_buf.
    Status status = spi_exchange(device, tx_buf, rx_buf_new, len + 1);
    memcpy(rx_buf, rx_buf_new + 1, len);

    DELAY_MICROS(100);

    return status;
}

static Status adxl372_write(SpiDevice* device, uint8_t address, uint8_t* tx_buf,
                            uint8_t len) {
    if (address > 127) {
        return STATUS_PARAMETER_ERROR;
    }

    // Create dummy read buffer
    uint8_t rx_buf[len + 1];

    // Create new tx buffer
    uint8_t tx_buf_new[len + 1];
    tx_buf_new[0] = address & 0x7F;  // Add address and write bit to tx buffer
    memcpy(tx_buf_new + 1, tx_buf,
           len);  // Copy bytes to end of the new tx buffer

    // Exchange address and write len bytes
    Status status = spi_exchange(device, tx_buf_new, rx_buf, len + 1);

    DELAY_MICROS(100);

    return status;
}

Status adxl372_init(SpiDevice* device, Adxl372Bandwidths bandwidth,
                    Adxl372OutputDataRate rate, Adxl372Modes mode) {
    uint8_t tx_buf;

    // reset sensor
    tx_buf = 0x52;
    if (adxl372_write(device, ADXL372_RESET, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Disable FIFO
    tx_buf = 0x00;
    if (adxl372_write(device, ADXL372_FIFO_CTL, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Configure the accel to 6.4khz rate
    tx_buf = rate;
    if (adxl372_write(device, ADXL372_TIMING, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // set bandwidth
    tx_buf = bandwidth;
    if (adxl372_write(device, ADXL372_MEASURE, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    // set mode of operation
    tx_buf = mode;
    if (adxl372_write(device, ADXL372_POWER_CTL, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

Accel adxl372_read_accel(SpiDevice* device) {
    Accel result;

    // Wait until the device is done writting any data to the registery
    uint8_t rx_write_check = 0x00;
    if (adxl372_read(device, ADXL372_STATUS, &rx_write_check, 1) != STATUS_OK ||
        !(rx_write_check & 0x01)) {
        result.accelX = NAN;
        result.accelY = NAN;
        result.accelZ = NAN;
        return result;
    }

    // Read all 6 registers at once
    uint8_t rx_buf[6];
    if (adxl372_read(device, ADXL372_DATA, rx_buf, 6) != STATUS_OK) {
        result.accelX = NAN;
        result.accelY = NAN;
        result.accelZ = NAN;
        return result;
    }

    // Convert unsigned 8-bit halves to signed 12-bit numbers
    int16_t acc_x_raw =
        ((int16_t)(((uint16_t)rx_buf[0] << 8) | (rx_buf[1] & 0xF0))) >> 4;
    int16_t acc_y_raw =
        ((int16_t)(((uint16_t)rx_buf[2] << 8) | (rx_buf[3] & 0xF0))) >> 4;
    int16_t acc_z_raw =
        ((int16_t)(((uint16_t)rx_buf[4] << 8) | (rx_buf[5] & 0xF0))) >> 4;

    float conversion_factor =
        100;  // this right? its .061 for the other accel at 1 LSB = .061 mg but
              // this one is 100mg/LSB

    float acc_x = ((float)acc_x_raw * conversion_factor) / 1000;
    float acc_y = ((float)acc_y_raw * conversion_factor) / 1000;
    float acc_z = ((float)acc_z_raw * conversion_factor) / 1000;

    result.accelX = acc_x;
    result.accelY = acc_y;
    result.accelZ = acc_z;

    return result;
}

Status adxl372_config(SpiDevice* device, Adxl372OutputDataRate rate) {
    uint8_t tx_buf = rate;

    // Configure the accelerometer to the specified measurement rate
    Status status = adxl372_write(device, ADXL372_TIMING, &tx_buf, 1);

    if (status != STATUS_OK) {
        return status;
    }

    uint8_t rx_buf;

    // Verify the settings
    status = adxl372_read(device, ADXL372_TIMING, &rx_buf, 1);

    if (status != STATUS_OK) {
        return status;
    }

    if ((rx_buf & 0xE0) != rate) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}