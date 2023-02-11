#include "ms5637.h"

#include "timer.h"

static uint32_t read_D1(I2cDevice* device, AdcSpeed speed);
static uint32_t read_D2(I2cDevice* device, AdcSpeed speed);

I2cDevice *device;
CalibrationData data;

uint8_t conversion_delay_ms[] = {1, 2, 3, 5, 9, 17};

Status ms5637_init(I2cDevice* device) {
    uint8_t rx_buf[2];
    uint8_t tx_buf[1] = {0x1E};

    // Send Reset Command
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };

    // Read Constants
    // Read C1
    tx_buf[0] = 0xA0 | (1 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C1 = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

    // Read C2
    tx_buf[0] = 0xA0 | (2 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C2 = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

    // Read C3
    tx_buf[0] = 0xA0 | (3 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C3 = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

    // Read C4
    tx_buf[0] = 0xA0 | (4 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C4 = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

    // Read C5
    tx_buf[0] = 0xA0 | (5 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C5 = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

    // Read C6
    tx_buf[0] = 0xA0 | (6 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C6 = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

    return OK;
}

static uint32_t read_D1(I2cDevice* device, AdcSpeed speed) {
    uint32_t D1 = 0;
    uint8_t rx_buf[3];
    uint8_t tx_buf[1] = {0x40 | speed};
    // Start ADC conversion
    if (i2c_write(device, tx_buf, 1) != OK) {
        return D_READ_ERROR;
    }
    DELAY(conversion_delay_ms[speed / 2] + 1);
    tx_buf[0] = 0x00;
    while (!D1) {
        // Send ADC read command
        if (i2c_write(device, tx_buf, 1) != OK) {
            return D_READ_ERROR;
        }
        if (i2c_read(device, rx_buf, 3) != OK) {
            return D_READ_ERROR;
        }
        D1 = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) |
             rx_buf[2];
    }
    return D1;
}

static uint32_t read_D2(I2cDevice* device, AdcSpeed speed) {
    uint32_t D2 = 0;
    uint8_t rx_buf[3];
    uint8_t tx_buf[1] = {0x50 | speed};
    // Start ADC conversion
    if (i2c_write(device, tx_buf, 1) != OK) {
        return D_READ_ERROR;
    }
    DELAY(conversion_delay_ms[speed / 2] + 1);
    tx_buf[0] = 0x00;
    while (!D2) {
        // Send ADC read command
        if (i2c_write(device, tx_buf, 1) != OK) {
            return D_READ_ERROR;
        }
        if (i2c_read(device, rx_buf, 3) != OK) {
            return D_READ_ERROR;
        }
        D2 = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) |
             rx_buf[2];
    }
    return D2;
}

BaroData ms5637_read(I2cDevice* device, AdcSpeed speed) {
    BaroData result;
    result.pressure = NAN;
    result.temperature = NAN;

    uint32_t D1;
    uint32_t D2;

    if ((D1 = read_D1(device, speed)) == D_READ_ERROR) {
        return result;
    }
    if ((D2 = read_D2(device, speed)) == D_READ_ERROR) {
        return result;
    }
    int32_t dT = D2 - (data.C5 * 256);
    int32_t TEMP = (int32_t)(2000 + (dT * ((float)data.C6 / 8388608)));
    int64_t OFF = ((int64_t)data.C2 << 17) + (((int64_t)data.C4 * dT) >> 6);
    int64_t SENS = ((int64_t)data.C1 << 16) + (((int64_t)data.C3 * dT) >> 7);
    int32_t T2;
    int64_t OFF2;
    int64_t SENS2;
    if (TEMP < 2000) {
        T2 = 3 * ((int64_t)dT * (int64_t)dT) >> 33;
        OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16;
        SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16;
        if (TEMP < -1500) {
            OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
            SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
        }
    } else {
        T2 = (5 * ((int64_t)dT * (int64_t)dT)) >> 38;
        OFF2 = 0;
        SENS2 = 0;
    }
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int64_t P = (((D1 * SENS) >> 21) - OFF) >> 15;

    result.pressure = (float)P / 100;
    result.temperature = (float)TEMP / 100;

    return result;
}
