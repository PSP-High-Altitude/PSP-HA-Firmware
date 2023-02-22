#include "i2c/i2c.h"

#include "imxrt1062/MIMXRT1062/drivers/fsl_clock.h"
#include "imxrt1062/MIMXRT1062/drivers/fsl_iomuxc.h"
#include "imxrt1062/MIMXRT1062/drivers/fsl_lpi2c.h"

static bool i2c_enabled[] = {0, 0, 0, 0};

static Status lpi2cSetup(I2cDevice *dev) {
    if (i2c_enabled[dev->periph]) {
        return STATUS_OK;
    }
    LPI2C_Type *base = NULL;
    switch (dev->periph) {
        case P_I2C1:
            base = LPI2C1;
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL, 1);  // pin 19
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA, 1);  // pin 18
            break;
        case P_I2C2:
            return STATUS_PARAMETER_ERROR;
            break;
        case P_I2C3:
            base = LPI2C3;
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_07_LPI2C3_SCL, 1);  // pin 16
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_LPI2C3_SDA, 1);  // pin 17
            break;
        case P_I2C4:
            base = LPI2C4;
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_LPI2C4_SCL, 1);  // pin 24
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_13_LPI2C4_SDA, 1);  // pin 25
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }
    lpi2c_master_config_t conf;
    LPI2C_MasterGetDefaultConfig(&conf);
    conf.baudRate_Hz = dev->clk;
    LPI2C_MasterInit(base, &conf, CLOCK_GetClockRootFreq(kCLOCK_Lpi2cClkRoot));
    i2c_enabled[dev->periph] = 1;
    return STATUS_OK;
}

Status i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len) {
    if (lpi2cSetup(device) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    lpi2c_master_transfer_t transfer = {
        .data = tx_buf,
        .dataSize = len,
        .direction = kLPI2C_Write,
        .slaveAddress = device->address,
    };
    LPI2C_Type *base = NULL;
    switch (device->periph) {
        case P_I2C1:
            base = LPI2C1;
            break;
        case P_I2C2:
            base = LPI2C2;
            break;
        case P_I2C3:
            base = LPI2C3;
            break;
        case P_I2C4:
            base = LPI2C4;
            break;
    }
    if (LPI2C_MasterTransferBlocking(base, &transfer) != kStatus_Success) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len) {
    if (lpi2cSetup(device) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    lpi2c_master_transfer_t transfer = {
        .data = rx_buf,
        .dataSize = len,
        .direction = kLPI2C_Read,
        .slaveAddress = device->address,
    };
    LPI2C_Type *base = NULL;
    switch (device->periph) {
        case P_I2C1:
            base = LPI2C1;
            break;
        case P_I2C2:
            base = LPI2C2;
            break;
        case P_I2C3:
            base = LPI2C3;
            break;
        case P_I2C4:
            base = LPI2C4;
            break;
    }
    if (LPI2C_MasterTransferBlocking(base, &transfer) != kStatus_Success) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}