#include "max_m10s.h"

#include "string.h"

static uint64_t* ubx_cfg_valget(I2cDevice*, Max_M10S_Layer_TypeDef, uint32_t*,
                                size_t);

Status max_m10s_init(I2cDevice* device) {}

static uint64_t* ubx_cfg_valget(I2cDevice* device, Max_M10S_Layer_TypeDef layer,
                                uint32_t* keys, size_t num_keys) {
    size_t tx_buf_len = 12 + 4 * num_keys;
    uint8_t* tx_buf = malloc(tx_buf_len);
    tx_buf[0] = 0xB5;
    tx_buf[1] = 0x62;
    tx_buf[2] = 0x06;
    tx_buf[3] = 0x8B;
    tx_buf[4] = ((4 * num_keys) >> 8) & 0xFF;
    tx_buf[5] = (4 * num_keys) & 0xFF;
    tx_buf[6] = 0;
    tx_buf[7] = (uint8_t)layer;
    tx_buf[8] = 0;
    tx_buf[9] = 0;
    for (uint16_t i = 0; i < num_keys; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            tx_buf[10 + (i * 4) + j] = (keys[i] >> (j * 8)) & 0xFF;
        }
    }
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for (uint16_t i = 0; i < tx_buf_len - 2; i++) {
        CK_A += tx_buf[i];
        CK_B += CK_A;
    }
    tx_buf[tx_buf_len - 2] = CK_A;
    tx_buf[tx_buf_len - 1] = CK_B;

    i2c_write(device, tx_buf, tx_buf_len);
    free(tx_buf);

    uint32_t message_header = 0;
    uint8_t* message_buf;
    uint16_t bytes_avail = 0;
    while (message_header != 0x8B0662B5) {
        bytes_avail = 0;
        while (!bytes_avail) {
            uint8_t addr_buf[1] = {0xFD};
            uint8_t rx_buf[2];
            i2c_write(device, addr_buf, 1);
            i2c_read(device, rx_buf, 2);
            bytes_avail = (rx_buf[0] << 8) + rx_buf[1];
        }
        message_buf = malloc(bytes_avail);
        i2c_read(device, message_buf, bytes_avail);
        for (uint8_t i = 0; i < 3; i++) {
            message_header = message_buf[i] << (i * 8);
        }
    }

    uint64_t values[num_keys];
    uint16_t message_idx = 10;
    for (uint8_t i = 0; i < num_keys; i++) {
        message_idx += 4;
        uint8_t val_size = (keys[i] >> 28) & 0x7;
        for (uint8_t val_idx = message_idx; val_idx < message_idx + val_size;
             val_idx++) {
            values[i] += message_buf[val_idx] << ((val_idx - message_idx) * 8);
        }
        message_idx += val_size;
    }

    free(message_buf);

    return values;
}