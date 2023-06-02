#ifndef I2C_H
#define I2C_H

#include <stddef.h>
#include <stdint.h>

#include "status.h"

typedef enum {
    P_I2C1 = 0,
    P_I2C2 = 1,
    P_I2C3 = 2,
    P_I2C4 = 3,
} I2cPeriph;

typedef enum {
    I2C_SPEED_INVALID = 0,
    I2C_SPEED_STANDARD = 100000,
    I2C_SPEED_FAST = 400000,
    I2C_SPEED_FAST_PLUS = 1000000,
} I2cSpeed;

typedef struct {
    uint8_t address;
    I2cSpeed clk;
    I2cPeriph periph;
} I2cDevice;

Status i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len);

Status i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len);

#endif // I2C_H
