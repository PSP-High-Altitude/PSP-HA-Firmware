#include "lsm6dsox.h"

#include <string.h>

#include "timer.h"

Lsm6dsoxAccelRange g_current_accel_range = LSM6DSOX_XL_RANGE_16_G;
Lsm6dsoxGyroRange g_current_gyro_range = LSM6DSOX_G_RANGE_2000_DPS;

static Status lsm6dsox_read(SpiDevice* device, uint8_t address, uint8_t* rx_buf,
                            uint8_t len) {
    if (address > 127) {
        return PARAMETER_ERROR;
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

static Status lsm6dsox_write(SpiDevice* device, uint8_t address,
                             uint8_t* tx_buf, uint8_t len) {
    if (address > 127) {
        return PARAMETER_ERROR;
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

Status lsm6dsox_init(SpiDevice* device) {
    uint8_t tx_buf;

    tx_buf = 0x85;
    lsm6dsox_write(device, LSM6DSOX_CTRL3_C, &tx_buf, 1);  // Reset

    // Disable I3C and DEN value
    tx_buf = 0x02;
    lsm6dsox_write(device, LSM6DSOX_CTRL9_XL, &tx_buf, 1);

    // Enable and configure the accel to 16g range and 6.66khz rate
    tx_buf = LSM6DSOX_XL_RANGE_16_G | LSM6DSOX_XL_RATE_6_66_KHZ;
    lsm6dsox_write(device, LSM6DSOX_CTRL1_XL, &tx_buf, 1);

    // Enable and configure the gyro to 2000dps range and 6.66khz rate
    tx_buf = LSM6DSOX_G_RANGE_2000_DPS | LSM6DSOX_G_RATE_6_66_KHZ;
    lsm6dsox_write(device, LSM6DSOX_CTRL2_G, &tx_buf, 1);

    return OK;
}

Accel lsm6dsox_read_accel(SpiDevice* device) {
    // Read all 6 registers at once
    uint8_t rx_buf[6];
    lsm6dsox_read(device, LSM6DSOX_OUT_A, rx_buf, 6);

    // Convert unsigned 8-bit halves to signed 16-bit numbers
    int16_t acc_x_raw = ((int16_t)(((uint16_t)rx_buf[1] << 8) | rx_buf[0]));
    int16_t acc_y_raw = ((int16_t)(((uint16_t)rx_buf[3] << 8) | rx_buf[2]));
    int16_t acc_z_raw = ((int16_t)(((uint16_t)rx_buf[5] << 8) | rx_buf[4]));

    // Convert data to units of g
    float conversion_factor = 0;
    switch (g_current_accel_range) {
        case LSM6DSOX_XL_RANGE_2_G:
            conversion_factor = 0.061;
            break;
        case LSM6DSOX_XL_RANGE_4_G:
            conversion_factor = 0.122;
            break;
        case LSM6DSOX_XL_RANGE_8_G:
            conversion_factor = 0.244;
            break;
        case LSM6DSOX_XL_RANGE_16_G:
            conversion_factor = 0.488;
            break;
    }

    float acc_x = ((float)acc_x_raw * conversion_factor) / 1000;
    float acc_y = ((float)acc_y_raw * conversion_factor) / 1000;
    float acc_z = ((float)acc_z_raw * conversion_factor) / 1000;

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
    lsm6dsox_read(device, LSM6DSOX_OUT_G, rx_buf, 6);

    // Convert unsigned 8-bit halves to signed 16-bit numbers
    int16_t g_x_raw = (int16_t)(((uint16_t)rx_buf[1] << 8) | rx_buf[0]);
    int16_t g_y_raw = (int16_t)(((uint16_t)rx_buf[3] << 8) | rx_buf[2]);
    int16_t g_z_raw = (int16_t)(((uint16_t)rx_buf[5] << 8) | rx_buf[4]);

    // Convert data to units of dps
    float conversion_factor = 0;
    switch (g_current_gyro_range) {
        case LSM6DSOX_G_RANGE_125_DPS:
            conversion_factor = 4.375;
            break;
        case LSM6DSOX_G_RANGE_250_DPS:
            conversion_factor = 8.75;
            break;
        case LSM6DSOX_G_RANGE_500_DPS:
            conversion_factor = 17.5;
            break;
        case LSM6DSOX_G_RANGE_1000_DPS:
            conversion_factor = 35.0;
            break;
        case LSM6DSOX_G_RANGE_2000_DPS:
            conversion_factor = 70.0;
            break;
    }

    float g_x = ((float)g_x_raw * conversion_factor) / 1000;
    float g_y = ((float)g_y_raw * conversion_factor) / 1000;
    float g_z = ((float)g_z_raw * conversion_factor) / 1000;

    Gyro result = {
        .gyroX = g_x,
        .gyroY = g_y,
        .gyroZ = g_z,
    };

    return result;
}

Status lsm6dsox_config_accel(SpiDevice* device, Lsm6dsoxAccelDataRate rate,
                             Lsm6dsoxAccelRange range) {
    uint8_t tx_buf = rate | range;

    Status status = lsm6dsox_write(device, LSM6DSOX_CTRL1_XL, &tx_buf,
                                   1);  // Configure the accelerometer to the
                                        // specified range and measurement rate
    if (status != OK) {
        return status;
    }

    uint8_t rx_buf;

    status = lsm6dsox_read(device, LSM6DSOX_CTRL1_XL, &rx_buf,
                           1);  // Verify the settings

    if (status != OK) {
        return status;
    }

    if ((rx_buf & 0xF0) != rate) {
        return rx_buf;
    }
    if ((rx_buf & 0x0C) != range) {
        return rx_buf;
    }
    g_current_accel_range = range;

    return OK;
}

Status lsm6dsox_config_gyro(SpiDevice* device, Lsm6dsoxGyroDataRate rate,
                            Lsm6dsoxGyroRange range) {
    uint8_t tx_buf = rate | range;
    if (range == LSM6DSOX_G_RANGE_125_DPS) {
        tx_buf |= 0x02;
    }

    Status status = lsm6dsox_write(device, LSM6DSOX_CTRL2_G, &tx_buf,
                                   1);  // Configure the gyroscope to the
                                        // specified range and measurement rate

    if (status != OK) {
        return status;
    }

    uint8_t rx_buf;

    status = lsm6dsox_read(device, LSM6DSOX_CTRL2_G, &rx_buf,
                           1);  // Verify the settings

    if (status != OK) {
        return status;
    }

    if ((rx_buf & 0xF0) != rate) {
        return ERROR;
    }
    if (range != LSM6DSOX_G_RANGE_125_DPS) {
        if ((rx_buf & 0x0C) != range) {
            return ERROR;
        }
    } else {
        if (!(rx_buf & 0x02)) {
            return ERROR;
        }
    }

    g_current_gyro_range = range;

    return OK;
}