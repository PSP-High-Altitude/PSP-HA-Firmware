#include "lsm6dsox.h"

#include <string.h>

static Status lsm6dsox_read(SpiDevice* device, uint8_t address, uint8_t* rx_buf,
                            uint8_t len) {
    if (address > 127) {
        return PARAMETER_ERROR;
    }

    if (len > 6) {
        return PARAMETER_ERROR;
    }

    address = address | 0x80;  // Add read bit to address

    uint8_t tx_buf[6] = {0};

    tx_buf[0] = address;

    return spi_exchange(device, tx_buf, rx_buf, len);
}

static Status lsm6dsox_write(SpiDevice* device, uint8_t address,
                             uint8_t* tx_buf, uint8_t len) {
    if (address > 127) {
        return PARAMETER_ERROR;
    }

    if (len > 6) {
        return PARAMETER_ERROR;
    }

    address = address & 0x7F;  // Add write bit to address

    uint8_t rx_buf[7];

    // Create new buffer with the address and copy over the old one
    uint8_t tx_buf_new[7];
    tx_buf_new[0] = address;
    memcpy(tx_buf_new + 1, tx_buf, len);

    return spi_exchange(device, tx_buf_new, rx_buf, len + 1);
}

Status lsm6dsox_init(SpiDevice* device) {
    uint8_t tx_buf = 0xA4;
    lsm6dsox_write(device, 0x10, &tx_buf,
                   1);  // Enable and configure the accelerometer to 16g range

    tx_buf = 0xAC;
    lsm6dsox_write(device, 0x10, &tx_buf,
                   1);  // Enable and configure the gyro to 2000dps range

    return OK;
}
Accel lsm6dsox_read_accel(SpiDevice* device) {
    // Read all 6 registers at once
    uint8_t rx_buf[6];
    lsm6dsox_read(device, 0x28, rx_buf, 6);

    // Convert unsigned 8-bit halves to signed 16-bit numbers
    int16_t acc_x_raw = (int16_t)(((uint16_t)rx_buf[1] << 8) | rx_buf[0]);
    int16_t acc_y_raw = (int16_t)(((uint16_t)rx_buf[3] << 8) | rx_buf[2]);
    int16_t acc_z_raw = (int16_t)(((uint16_t)rx_buf[5] << 8) | rx_buf[4]);

    // Convert data to units of g
    float acc_x = (acc_x_raw * 0.488) / 1000;
    float acc_y = (acc_y_raw * 0.488) / 1000;
    float acc_z = (acc_z_raw * 0.488) / 1000;

    Accel result = {
        .accelX = acc_x,
        .accelY = acc_y,
        .accelZ = acc_z,
    };

    return result;
}
Gyro lsm6dsox_read_gyro(SpiDevice* device) {
    // Read all 6 registers at once
    uint8_t rx_buf[6];
    lsm6dsox_read(device, 0x22, rx_buf, 6);

    // Convert unsigned 8-bit halves to signed 16-bit numbers
    int16_t g_x_raw = (int16_t)(((uint16_t)rx_buf[1] << 8) | rx_buf[0]);
    int16_t g_y_raw = (int16_t)(((uint16_t)rx_buf[3] << 8) | rx_buf[2]);
    int16_t g_z_raw = (int16_t)(((uint16_t)rx_buf[5] << 8) | rx_buf[4]);

    // Convert data to units of dps
    float g_x = (g_x_raw * 70) / 1000;
    float g_y = (g_y_raw * 70) / 1000;
    float g_z = (g_z_raw * 70) / 1000;

    Gyro result = {
        .gyroX = g_x,
        .gyroY = g_y,
        .gyroZ = g_z,
    };

    return result;
}