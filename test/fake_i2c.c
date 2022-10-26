#include "i2c/i2c.h"

Status i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len) { return OK; }

Status i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        rx_buf[i] = 0xff;
    }
    return OK;
}
