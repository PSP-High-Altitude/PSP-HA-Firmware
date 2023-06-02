#include "iis2mdc.h"

#include <math.h>
#include <string.h>

#include "timer.h"

Status iis2mdc_init(I2cDevice* device, Iis2mdcODR odr) {
    uint8_t buf[2];

    // Read WHO_AM_I register to confirm we're connected
    buf[0] = IIS2MDC_WHO_AM_I;
    if (i2c_write(device, buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (i2c_read(device, buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (buf[0] != 0b01000000) {
        return STATUS_ERROR;
    }

    // Perform configuration
    buf[0] = IIS2MDC_CFG_A;
    buf[1] = ((1 << 7) |     // Enable temperature compensation
              (odr << 2) |   // Set output data rate from args
              (0b00 << 0));  // Set operation mode to continuous
    if (i2c_write(device, buf, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

Mag iis2mdc_read(I2cDevice* device) {
    uint8_t buf[6];
    Mag mag = {NAN, NAN, NAN};

    buf[0] = IIS2MDC_OUT | 0x80;  // Set MSb for auto increment
    if (i2c_write(device, buf, 1) != STATUS_OK) {
        return mag;
    }
    if (i2c_read(device, buf, 6) != STATUS_OK) {
        return mag;
    }

    mag.magX =
        (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]) * 1.5 / 1000;
    mag.magY =
        (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]) * 1.5 / 1000;
    mag.magZ =
        (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]) * 1.5 / 1000;

    return mag;
}
