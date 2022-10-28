#include "ms5637.h"

static uint32_t read_D1(AdcSpeed speed);
static uint32_t read_D2(AdcSpeed speed);

I2cDevice *device;
CalibrationData data;

Status ms5637_init(I2cDevice* device) {
    uint8_t rx_buf[2];
    uint8_t tx_buf[1] = {0x1E};

    // Send Reset Command
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };

    // Read Constants
    tx_buf[0] = 0xA0;

    // Read C1
    tx_buf[0] |= (1 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C1 = (uint16_t)rx_buf[0] | rx_buf[1];

    // Read C2
    tx_buf[0] |= (2 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C2 = (uint16_t)rx_buf[0] | rx_buf[1];

    // Read C3
    tx_buf[0] |= (3 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C3 = (uint16_t)rx_buf[0] | rx_buf[1];

    // Read C4
    tx_buf[0] |= (4 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C4 = (uint16_t)rx_buf[0] | rx_buf[1];

    // Read C5
    tx_buf[0] |= (5 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C5 = (uint16_t)rx_buf[0] | rx_buf[1];

    // Read C6
    tx_buf[0] |= (6 << 1);
    if (i2c_write(device, tx_buf, 1) != OK) {
        return ERROR;
    };
    if (i2c_read(device, rx_buf, 2) != OK) {
        return ERROR;
    };
    data.C6 = (uint16_t)rx_buf[0] | rx_buf[1];

    return OK;
}

static uint32_t read_D1(AdcSpeed speed) {
    uint8_t rx_buf[2] = {0, 0};
    uint8_t tx_buf[1] = {0x40 | speed};
    if (i2c_write(device, tx_buf, 1) != OK) {
        return D_READ_ERROR;
    }
    while ((uint16_t)rx_buf[0] | rx_buf[1]) {
        if (i2c_read(device, rx_buf, 2) != OK) {
            return D_READ_ERROR;
        }
    }
    uint32_t D1 = (uint16_t)rx_buf[0] | rx_buf[1];
    return D1;
}

static uint32_t read_D2(AdcSpeed speed) {
    uint8_t rx_buf[2] = {0, 0};
    uint8_t tx_buf[1] = {0x50 | speed};
    if (i2c_write(device, tx_buf, 1) != OK) {
        return D_READ_ERROR;
    }
    while ((uint16_t)rx_buf[0] | rx_buf[1]) {
        if (i2c_read(device, rx_buf, 2) != OK) {
            return D_READ_ERROR;
        }
    }
    uint32_t D2 = (uint16_t)rx_buf[0] | rx_buf[1];
    return D2;
}

BaroData ms5637_read(AdcSpeed speed) {
    BaroData result;
    result.pressure = NAN;
    result.temperature = NAN;

    uint32_t D1;
    uint32_t D2;

    if ((D1 = read_D1(speed)) == D_READ_ERROR) {
        return result;
    }
    if ((D2 = read_D2(speed)) == D_READ_ERROR) {
        return result;
    }
    int32_t dT = D2 - (data.C5 * 256);
    int32_t TEMP = (int32_t)(2000 + (dT * ((float)data.C6 / 8388608)));
    int64_t OFF = (data.C2 * 131072) + ((data.C4 * dT) / 64);
    int64_t SENS = (data.C1 * 65536) + ((data.C3 * dT) / 128);
    int32_t T2;
    int64_t OFF2;
    int64_t SENS2;
    if ((float)TEMP / 100 < 20.00) {
        T2 = 3 * (dT * dT) / 8589934592;
        OFF2 = 61 * (TEMP - 2000) * (TEMP - 2000) / 16;
        SENS2 = 29 * (TEMP - 2000) * (TEMP - 2000) / 16;
        if ((float)TEMP / 100 < -15.00) {
            OFF2 += 17 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 9 * (TEMP + 1500) * (TEMP + 1500);
        }
    } else {
        T2 = 5 * (dT * dT) / 274877906944;
        OFF2 = 0;
        SENS2 = 0;
    }
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int32_t P = ((D1 * SENS / 2097152) - OFF) / 32768;

    result.pressure = (float)P / 100;
    result.temperature = (float)TEMP / 100;

    return result;
}