#include "bmi088_model.h"

#include "string.h"

uint8_t reset = 0;
uint8_t acc_regs[0xFF];
uint8_t gyr_regs[0xFF];
uint8_t selected_acc_reg = 0;
uint8_t selected_gyr_reg = 0;

Status bmi088_model_i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len) {
    if (!reset) {
        memset(acc_regs, 0, sizeof(acc_regs));
        memset(gyr_regs, 0, sizeof(gyr_regs));

        acc_regs[BMI088_ACC_CHIP_ID] = 0x1E;
        acc_regs[BMI088_ACC_CONF] = 0xA8;
        acc_regs[BMI088_ACC_RANGE] = 0x01;

        gyr_regs[BMI088_GYR_CHIP_ID] = 0x0F;
        gyr_regs[BMI088_GYR_RANGE] = 0x00;
        gyr_regs[BMI088_GYR_BANDWIDTH] = 0x80;

        double x_magic = -1.2345;
        double y_magic = 5.5555;
        double z_magic = 3.1415;

        int16_t x = x_magic * 1365;
        int16_t y = y_magic * 1365;
        int16_t z = z_magic * 1365;

        acc_regs[BMI088_ACC_DATA] = x & 0xFF;
        acc_regs[BMI088_ACC_DATA + 1] = (x >> 8) & 0xFF;
        acc_regs[BMI088_ACC_DATA + 2] = y & 0xFF;
        acc_regs[BMI088_ACC_DATA + 3] = (y >> 8) & 0xFF;
        acc_regs[BMI088_ACC_DATA + 4] = z & 0xFF;
        acc_regs[BMI088_ACC_DATA + 5] = (z >> 8) & 0xFF;

        x_magic = -1234.5;
        y_magic = 555.55;
        z_magic = 314.15;

        x = x_magic * 16.384;
        y = y_magic * 16.384;
        z = z_magic * 16.384;

        gyr_regs[BMI088_GYR_DATA] = x & 0xFF;
        gyr_regs[BMI088_GYR_DATA + 1] = (x >> 8) & 0xFF;
        gyr_regs[BMI088_GYR_DATA + 2] = y & 0xFF;
        gyr_regs[BMI088_GYR_DATA + 3] = (y >> 8) & 0xFF;
        gyr_regs[BMI088_GYR_DATA + 4] = z & 0xFF;
        gyr_regs[BMI088_GYR_DATA + 5] = (z >> 8) & 0xFF;

        reset = 1;
    }

    if (device->address != BMI088_ACC_I2C_ADDR &&
        device->address != BMI088_GYR_I2C_ADDR) {
        // This function shouldn't have been called with any other address
        return STATUS_ERROR;
    }

    if (device->clk != I2C_SPEED_STANDARD && device->clk != I2C_SPEED_FAST) {
        // The device only supports standard and fast speed modes
        return STATUS_ERROR;
    }

    if (len > 2) {
        return STATUS_ERROR;
    }

    if (len == 1) {
        if (device->address == BMI088_ACC_I2C_ADDR) {
            selected_acc_reg = tx_buf[0];
        } else {
            selected_gyr_reg = tx_buf[0];
        }
        return STATUS_OK;
    }

    if (device->address == BMI088_ACC_I2C_ADDR) {
        selected_acc_reg = tx_buf[0];
        acc_regs[selected_acc_reg] = tx_buf[1];
    } else {
        selected_gyr_reg = tx_buf[0];
        gyr_regs[selected_gyr_reg] = tx_buf[1];
    }

    return STATUS_OK;
}

Status bmi088_model_i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len) {
    if (device->address != BMI088_ACC_I2C_ADDR &&
        device->address != BMI088_GYR_I2C_ADDR) {
        // This function shouldn't have been called with any other address
        return STATUS_ERROR;
    }

    if (device->clk != I2C_SPEED_STANDARD && device->clk != I2C_SPEED_FAST) {
        // The device only supports standard and fast speed modes
        return STATUS_ERROR;
    }

    for (int i = 0; i < len; i++) {
        if (device->address == BMI088_ACC_I2C_ADDR) {
            rx_buf[i] = acc_regs[selected_acc_reg++];
        } else {
            rx_buf[i] = gyr_regs[selected_gyr_reg++];
        }
    }

    return STATUS_OK;
}