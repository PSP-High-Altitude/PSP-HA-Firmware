#include "ms5637_model.h"

static const uint8_t CMD_RESET = 0x1E;
static const uint8_t CMD_CONV_D1_PREFIX = 0x40;
static const uint8_t CMD_CONV_D2_PREFIX = 0x50;
static const uint8_t CMD_READ_ADC = 0x00;
static const uint8_t CMD_READ_PROM_PREFIX = 0xA0;

static uint8_t s_command_history[2] = {0xFF};

// All of these defaults are example values from the datasheet
static uint32_t s_model_d1 = 6465444;
static uint32_t s_model_d2 = 8077636;
static uint16_t s_model_prom[7] = {
    32768 /* CRC calculated using vendor-provided function */,
    46372 /* C1 */,
    43981 /* C2 */,
    29059 /* C3 */,
    27842 /* C4 */,
    31553 /* C5 */,
    28165 /* C6 */,
};

Status ms5637_model_i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len) {
    if (device->address != MS5637_I2C_ADDR) {
        // This function shouldn't have been called with any other address
        return STATUS_ERROR;
    }

    if (device->clk != I2C_SPEED_STANDARD && device->clk != I2C_SPEED_FAST) {
        // The device only supports standard and fast speed modes
        return STATUS_ERROR;
    }

    if (len != 1) {
        // In theory the device should return a NACK for an invalid non-1 byte
        // write and the peripheral should raise an error (not tested)
        return STATUS_ERROR;
    }

    if (tx_buf[0] == CMD_RESET) {
        // If we get a reset command, clear the command history and return
        s_command_history[0] = 0xFF;
        s_command_history[1] = 0xFF;
        return STATUS_OK;
    }

    // Save the current command in the history buffer
    s_command_history[1] = s_command_history[0];
    s_command_history[0] = tx_buf[0];

    return STATUS_OK;
}

Status ms5637_model_i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len) {
    if (device->address != MS5637_I2C_ADDR) {
        // This function shouldn't have been called with any other address
        return STATUS_ERROR;
    }

    if (device->clk != I2C_SPEED_STANDARD && device->clk != I2C_SPEED_FAST) {
        // The device only supports standard and fast speed modes
        return STATUS_ERROR;
    }

    // What we return depends on the previous commands
    if (s_command_history[0] == CMD_READ_ADC) {
        // Currently we don't model the delay between sending the conversion
        // command and value becoming available through the ADC read command,
        // and so it doesn't matter which conversion command is used
        if (len != 3) {
            return STATUS_ERROR;
        }
        if ((s_command_history[1] & 0xF0) == CMD_CONV_D1_PREFIX) {
            rx_buf[0] = (s_model_d1 >> 16) & 0xFF;
            rx_buf[1] = (s_model_d1 >> 8) & 0xFF;
            rx_buf[2] = (s_model_d1 >> 0) & 0xFF;
        } else if ((s_command_history[1] & 0xF0) == CMD_CONV_D2_PREFIX) {
            rx_buf[0] = (s_model_d2 >> 16) & 0xFF;
            rx_buf[1] = (s_model_d2 >> 8) & 0xFF;
            rx_buf[2] = (s_model_d2 >> 0) & 0xFF;
        } else {
            return STATUS_ERROR;
        }
    } else if ((s_command_history[0] & 0xF0) == CMD_READ_PROM_PREFIX) {
        size_t prom_addr = (s_command_history[0] >> 1) & 0b111;
        if (prom_addr > 6) {
            return STATUS_ERROR;
        }
        if (len != 2) {
            return STATUS_ERROR;
        }
        rx_buf[0] = (s_model_prom[prom_addr] >> 8) & 0xFF;
        rx_buf[1] = (s_model_prom[prom_addr] >> 0) & 0xFF;
    }

    return STATUS_OK;
}

// NOTE: this is a vendor-provided function for calculating the CRC stored in
// the PROM. At some point we may want to implement our own version of this
// function, but it's probably not worth the effort right now.
/*
static unsigned char crc4(unsigned int n_prom[]) {
    int cnt;                 // simple counter
    unsigned int n_rem = 0;  // crc reminder
    unsigned char n_bit;
    n_prom[0] = ((n_prom[0]) & 0x0FFF);  // CRC byte is replaced by 0
    n_prom[7] = 0;                       // Subsidiary value, set to 0
    for (cnt = 0; cnt < 16; cnt++)       // operation is performed on bytes
    {                                    // choose LSB or MSB
        if (cnt % 2 == 1)
            n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else
            n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);  // final 4-bit reminder is CRC code
    return (n_rem ^ 0x00);
}
*/
