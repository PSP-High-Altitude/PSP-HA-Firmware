#include "i2c/i2c.h"

#include "imxrt1062/MIMXRT1062/drivers/fsl_lpi2c.h"

static bool i2c_enabled[] = {0, 0, 0, 0};

static Status lpi2cSetup(I2cDevice *dev) {
    if (!i2c_enabled[dev->periph]) {
        return OK;
    }
    LPI2C_Type *base = NULL;
    switch (dev->periph) {
        case I2C0:
            base = LPI2C1;
            break;
        case I2C1:
            base = LPI2C2;
            break;
        case I2C2:
            base = LPI2C3;
            break;
        case I2C3:
            base = LPI2C4;
            break;
    }
    lpi2c_master_config_t conf;
    LPI2C_MasterGetDefaultConfig(&conf);
    conf.baudRate_Hz = dev->clk;
    LPI2C_MasterInit(base, &conf, LPI2C_SRC_CLK);
    i2c_enabled[dev->periph] = 1;
    return OK;
}

Status i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len) {
    lpi2cSetup(device);
    lpi2c_master_transfer_t transfer = {
        .data = tx_buf,
        .dataSize = len,
        .direction = kLPI2C_Write,
        .slaveAddress = device->address,
    };
    LPI2C_Type *base = NULL;
    switch (device->periph) {
        case I2C0:
            base = LPI2C1;
            break;
        case I2C1:
            base = LPI2C2;
            break;
        case I2C2:
            base = LPI2C3;
            break;
        case I2C3:
            base = LPI2C4;
            break;
    }
    if (LPI2C_MasterTransferBlocking(base, &transfer) != kStatus_Success) {
        return ERROR;
    }
    return OK;
}

Status i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len) {
    lpi2cSetup(device);
    lpi2c_master_transfer_t transfer = {
        .data = rx_buf,
        .dataSize = len,
        .direction = kLPI2C_Read,
        .slaveAddress = device->address,
    };
    LPI2C_Type *base = NULL;
    switch (device->periph) {
        case I2C0:
            base = LPI2C1;
            break;
        case I2C1:
            base = LPI2C2;
            break;
        case I2C2:
            base = LPI2C3;
            break;
        case I2C3:
            base = LPI2C4;
            break;
    }
    if (LPI2C_MasterTransferBlocking(base, &transfer) != kStatus_Success) {
        return ERROR;
    }
    return OK;
}