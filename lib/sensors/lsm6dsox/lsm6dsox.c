#include "lsm6sdox.h"

SpiDevice* device;

static Status lsm6dsox_read(SpiDevice* device, uint8_t address, uint8_t* rx_buf,
                            uint8_t len) {
    if (address > 127) {
        return PARAMETER_ERROR;
    }

    if (len > 6) {
        return PARAMETER_ERROR;
    }

    address = (address << 1) | 1;  // Add read bit to address

    uint8_t tx_buf[6] = {0};

    tx_buf[0] = address;

    return SpiExchange(device, tx_buf, rx_buf, len);
}

static Status lsm6dsox_write(SpiDevice* device, uint8_t address,
                             uint8_t* tx_buf, uint8_t len) {
    if (address > 127) {
        return PARAMETER_ERROR;
    }

    if (len > 6) {
        return PARAMETER_ERROR;
    }

    address = (address << 1) | 0;  // Add read bit to address

    uint8_t rx_buf[7];

    uint8_t tx_buf_new[7];

    tx_buf_new[0] = address;
    for (int i = 1; i < len + 1; i++) {
        tx_buf_new[i] = tx_buf[i - 1];
    }

    return SpiExchange(device, tx_buf_new, rx_buf, len + 1);
}
