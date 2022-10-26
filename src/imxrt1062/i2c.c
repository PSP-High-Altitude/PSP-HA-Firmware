#include "i2c/i2c.h"

#include "registers.h"

static I2cDevice g_i2c_state[4] = {
    {
        .address = 0,
        .clk = I2C_SPEED_INVALID,
        .periph = I2C0,
    },
    {
        .address = 0,
        .clk = I2C_SPEED_INVALID,
        .periph = I2C1,
    },
    {
        .address = 0,
        .clk = I2C_SPEED_INVALID,
        .periph = I2C2,
    },
    {
        .address = 0,
        .clk = I2C_SPEED_INVALID,
        .periph = I2C3,
    },
};

static LPI2C_t *g_i2c_base[4] = {LPI2C1, LPI2C2, LPI2C3, LPI2C4};

static Status lpi2cSetup(I2cDevice *dev) {
    if (dev->clk == I2C_SPEED_STANDARD) {
        LPI2C_t *i2cBase = g_i2c_base[dev->periph];

        CCM->CCGR2 |= 0x000000C0;  // Enable the clock gate for LPI2C1

        PIN18_MUX &= 0xFFFFFFF0;
        PIN18_MUX |= 0x00000003;
        PIN19_MUX &= 0xFFFFFFF0;
        PIN19_MUX |= 0x00000003;

        i2cBase->MCR |= 0x00000002;     // Reset I2C
        i2cBase->MCR &= 0;              // Clear reset I2C
        i2cBase->MCFGR1 |= 0x00000003;  // PRESCALE: 3 (from 60MHz base)
        i2cBase->MCFGR2 |= 0x04000000;  // FILTSDA: 4, FILTSCL: 0, BUSIDLE: 0
        i2cBase->MCCR0 |=
            0x11242326;  // DATAVD: 17, SETHOLD: 36, CLKHI: 35, CLKLO: 38
        i2cBase->MCR |= 0x00000001;  // Enable I2C

        g_i2c_state[dev->periph].clk = dev->clk;
        return OK;
    } else if (dev->clk == I2C_SPEED_FAST) {
        LPI2C_t *i2cBase = g_i2c_base[dev->periph];

        CCM->CCGR2 |= 0x000000C0;  // Enable the clock gate for LPI2C1

        PIN18_MUX &= 0xFFFFFFF0;
        PIN18_MUX |= 0x00000003;
        PIN19_MUX &= 0xFFFFFFF0;
        PIN19_MUX |= 0x00000003;

        i2cBase->MCR |= 0x00000002;     // Reset I2C
        i2cBase->MCR &= 0;              // Clear reset I2C
        i2cBase->MCFGR1 |= 0x00000001;  // PRESCALE: 1 (from 60MHz base)
        i2cBase->MCFGR2 |= 0x03000000;  // FILTSDA: 3, FILTSCL: 0, BUSIDLE: 0
        i2cBase->MCCR0 |=
            0x11242226;  // DATAVD: 17, SETHOLD: 36, CLKHI: 34, CLKLO: 38
        i2cBase->MCR |= 0x00000001;  // Enable I2C

        g_i2c_state[dev->periph].clk = dev->clk;
        return OK;
    } else if (dev->clk == I2C_SPEED_FAST_PLUS) {
        LPI2C_t *i2cBase = g_i2c_base[dev->periph];

        CCM->CCGR2 |= 0x000000C0;  // Enable the clock gate for LPI2C1

        PIN18_MUX &= 0xFFFFFFF0;
        PIN18_MUX |= 0x00000003;
        PIN19_MUX &= 0xFFFFFFF0;
        PIN19_MUX |= 0x00000003;

        i2cBase->MCR |= 0x00000002;     // Reset I2C
        i2cBase->MCR &= 0;              // Clear reset I2C
        i2cBase->MCFGR1 |= 0x00000000;  // PRESCALE: 0 (from 60MHz base)
        i2cBase->MCFGR2 |= 0x00000000;  // FILTSDA: 0, FILTSCL: 0, BUSIDLE: 0
        i2cBase->MCCR0 |=
            0x0E1D1E;  // DATAVD: 14, SETHOLD: 29, CLKHI: 26, CLKLO: 30
        i2cBase->MCR |= 0x00000001;  // Enable I2C

        g_i2c_state[dev->periph].clk = dev->clk;
        return OK;
    }
    return ERROR;
}

Status i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len) {
    if (device->clk != g_i2c_state[device->periph].clk) {
        if (lpi2cSetup(device) != OK) {
            return ERROR;
        }
    }
    LPI2C_t *i2cBase = g_i2c_base[device->periph];

    if ((i2cBase->MSR) & 0x01000000) {
        return BUSY;
    }  // return busy if the bus is in use
    i2cBase->MTDR =
        0x00000400 |
        (device->address
         << 1);  // Set cmd to generate start condition and send address
    for (int i = 0; i < len; i++) {
        i2cBase->MTDR = (uint32_t) * (tx_buf + i);  // Send remaining data bytes
    }
    i2cBase->MTDR = 0x00000200;  // Generate stop condition
    while ((i2cBase->MSR) & 0x01000000) {
    }  // Wait for master idle
    return OK;
}

Status i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len) {
    if (device->clk != g_i2c_state[device->periph].clk) {
        if (lpi2cSetup(device) != OK) {
            return ERROR;
        }
    }
    LPI2C_t *i2cBase = g_i2c_base[device->periph];

    while ((i2cBase->MSR) & 0x01000000) {
    }  // Wait for master idle
    i2cBase->MTDR = 0x00000400 | (device->address << 1) |
                    1;  // Set cmd to generate start condition and send address
    i2cBase->MTDR = 0x00000100 | (len - 1);  // Receive numBytes bytes
    i2cBase->MTDR = 0x00000200;              // Generate stop condition
    while ((i2cBase->MSR) & 0x01000000)
        ;  // Wait for master idle
    for (int i = 0; i < len; i++) {
        *(rx_buf + i) =
            (uint8_t)(i2cBase->MRDR & 0xFF);  // Read out received bytes
                                              // into the provided pointer
    }
    return OK;
}