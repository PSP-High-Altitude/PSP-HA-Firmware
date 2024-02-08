#include "max_m10s.h"

#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "string.h"
#include "task.h"
#include "timer.h"

static Status ubx_cfg_valget(I2cDevice*, Max_M10S_Layer_TypeDef, uint32_t*,
                             size_t, uint64_t*);

static Status ubx_cfg_valset(I2cDevice* device, Max_M10S_Layer_TypeDef layer,
                             uint32_t* keys, uint64_t* values,
                             uint8_t* value_lens, size_t num_items);

Status max_m10s_init(I2cDevice* device) {
    uint32_t keys[] = {0x20110021, 0x10720002, 0x209100ba,
                       0x209100c9, 0x209100bf, 0x209100c4,
                       0x209100ab, 0x209100b0, 0x30210001};
    uint64_t values[] = {8, 0, 0, 0, 0, 0, 0, 0, 100};
    uint8_t value_lens[] = {1, 1, 1, 1, 1, 1, 1, 1, 2};
    if (ubx_cfg_valset(device, MAX_M10S_LAYER_SET_RAM, keys, values, value_lens,
                       9) == STATUS_OK) {
        printf("Set nav mode to airborne <4G\n");
        printf("Disabled NMEA on I2C\n");
        printf("Navigation measurement rate set to 10 Hz\n");
    } else {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

static Status ubx_read_msg(I2cDevice* device, uint8_t header[4],
                           uint8_t* message_buf, uint16_t* message_len,
                           uint32_t timeout) {
    uint8_t header_idx = 0;
    uint64_t start_time = MILLIS();

    // Read until we get a matching header
    while (1) {
        uint8_t tx_buf[1] = {0xFF};
        uint8_t rx_buf[1] = {0xFF};
        if (MILLIS() - start_time > timeout) {
            return STATUS_TIMEOUT_ERROR;
        }
        while (rx_buf[0] == 0xFF) {
            if (MILLIS() - start_time > timeout) {
                return STATUS_TIMEOUT_ERROR;
            }

            if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
                taskYIELD();
            }

            if (i2c_write(device, tx_buf, 1) != STATUS_OK) {
                return STATUS_ERROR;
            }
            if (i2c_read(device, rx_buf, 1) != STATUS_OK) {
                return STATUS_ERROR;
            }
        }
        if (rx_buf[0] == header[header_idx]) {
            if (header_idx == 3) {
                break;
            }
            header_idx++;
            continue;
        }
    }

    // Read out the length bytes
    uint8_t tx_buf[1] = {0xFF};
    uint8_t rx_buf[2] = {0xFF, 0xFF};
    if (i2c_write(device, tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (i2c_read(device, rx_buf, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (rx_buf[0] == 0xFF && rx_buf[1] == 0xFF) {
        return STATUS_ERROR;
    }
    uint16_t len = rx_buf[0] + ((uint16_t)rx_buf[1] << 8);
    *message_len = len;

    // Read out the payload bytes
    if (i2c_write(device, tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (i2c_read(device, message_buf, len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Read out the checksum bytes
    if (i2c_write(device, tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (i2c_read(device, rx_buf, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Check checksum
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for (uint8_t i = 2; i < 4; i++) {
        CK_A += header[i];
        CK_B += CK_A;
    }
    CK_A += (uint8_t)(len & 0xFF);
    CK_B += CK_A;
    CK_A += (uint8_t)((len >> 8) & 0xFF);
    CK_B += CK_A;
    for (uint16_t i = 0; i < len; i++) {
        CK_A += message_buf[i];
        CK_B += CK_A;
    }
    if (CK_A == rx_buf[0] && CK_B == rx_buf[1]) {
        return STATUS_OK;
    }
    return STATUS_ERROR;
}

__attribute__((unused)) static Status ubx_cfg_valget(
    I2cDevice* device, Max_M10S_Layer_TypeDef layer, uint32_t* keys,
    size_t num_keys, uint64_t* values) {
    size_t tx_buf_len = 12 + 4 * num_keys;
    uint8_t tx_buf[tx_buf_len];
    tx_buf[0] = 0xB5;
    tx_buf[1] = 0x62;
    tx_buf[2] = 0x06;
    tx_buf[3] = 0x8B;
    tx_buf[4] = (4 + 4 * num_keys) & 0xFF;
    tx_buf[5] = ((4 + 4 * num_keys) >> 8) & 0xFF;
    tx_buf[6] = 0;
    tx_buf[7] = (uint8_t)layer;
    tx_buf[8] = 0;
    tx_buf[9] = 0;
    for (uint16_t i = 0; i < num_keys; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            tx_buf[10 + (i * 4) + j] = (keys[i] >> (j * 8)) & 0xFF;
        }
    }
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for (uint16_t i = 2; i < tx_buf_len - 2; i++) {
        CK_A += tx_buf[i];
        CK_B += CK_A;
    }
    tx_buf[tx_buf_len - 2] = CK_A;
    tx_buf[tx_buf_len - 1] = CK_B;

    if (i2c_write(device, tx_buf, tx_buf_len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    uint8_t message_header[] = {0xB5, 0x62, 0x06, 0x8B};
    uint8_t message_buf[800];
    uint16_t message_len = 0;

    if (ubx_read_msg(device, message_header, message_buf, &message_len, 1000) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    uint16_t message_idx = 0;
    for (uint8_t i = 0; i < num_keys; i++) {
        message_idx += 4;
        uint8_t val_size = (keys[i] >> 28) & 0x7;
        for (uint8_t val_idx = message_idx; val_idx < message_idx + val_size;
             val_idx++) {
            values[i] += message_buf[val_idx] << ((val_idx - message_idx) * 8);
        }
        message_idx += val_size;
    }

    return STATUS_OK;
}

static Status ubx_cfg_valset(I2cDevice* device, Max_M10S_Layer_TypeDef layer,
                             uint32_t* keys, uint64_t* values,
                             uint8_t* value_lens, size_t num_items) {
    uint16_t total_len = 0;
    for (uint8_t i = 0; i < num_items; i++) {
        total_len += 4 + value_lens[i];
    }
    size_t tx_buf_len = 12 + total_len;
    uint8_t tx_buf[tx_buf_len];
    tx_buf[0] = 0xB5;
    tx_buf[1] = 0x62;
    tx_buf[2] = 0x06;
    tx_buf[3] = 0x8A;
    tx_buf[4] = (4 + total_len) & 0xFF;
    tx_buf[5] = ((4 + total_len) >> 8) & 0xFF;
    tx_buf[6] = 1;
    tx_buf[7] = (uint8_t)layer;
    tx_buf[8] = 0;
    tx_buf[9] = 0;
    uint16_t tx_buf_idx = 10;
    for (uint16_t i = 0; i < num_items; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            tx_buf[tx_buf_idx] = (keys[i] >> (j * 8)) & 0xFF;
            tx_buf_idx += 1;
        }
        for (uint8_t j = 0; j < value_lens[i]; j++) {
            tx_buf[tx_buf_idx] = (values[i] >> (j * 8)) & 0xFF;
            tx_buf_idx += 1;
        }
    }
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for (uint16_t i = 2; i < tx_buf_len - 2; i++) {
        CK_A += tx_buf[i];
        CK_B += CK_A;
    }
    tx_buf[tx_buf_len - 2] = CK_A;
    tx_buf[tx_buf_len - 1] = CK_B;

    for (uint16_t i = 0; i < tx_buf_len; i++) {
    }
    if (i2c_write(device, tx_buf, tx_buf_len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    uint8_t message_header[] = {0xB5, 0x62, 0x05, 0x01};
    uint8_t message_buf[800];
    uint16_t message_len = 0;

    if (ubx_read_msg(device, message_header, message_buf, &message_len, 1000) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    if (message_buf[0] == 0x06 && message_buf[1] == 0x8A) {
        return STATUS_OK;
    } else {
        return STATUS_ERROR;
    }
}

Status max_m10s_poll_fix(I2cDevice* device, GPS_Fix_TypeDef* fix) {
    size_t tx_buf_len = 8;
    uint8_t tx_buf[tx_buf_len];
    tx_buf[0] = 0xB5;
    tx_buf[1] = 0x62;
    tx_buf[2] = 0x01;
    tx_buf[3] = 0x07;
    tx_buf[4] = 0;
    tx_buf[5] = 0;

    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for (uint16_t i = 2; i < tx_buf_len - 2; i++) {
        CK_A += tx_buf[i];
        CK_B += CK_A;
    }
    tx_buf[tx_buf_len - 2] = CK_A;
    tx_buf[tx_buf_len - 1] = CK_B;

    if (i2c_write(device, tx_buf, tx_buf_len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    uint8_t message_header[] = {0xB5, 0x62, 0x01, 0x07};
    uint8_t message_buf[800];
    uint16_t message_len = 0;

    if (ubx_read_msg(device, message_header, message_buf, &message_len, 1000) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    if (message_len != 92) {
        return STATUS_ERROR;
    }

    fix->year = message_buf[4] + (message_buf[5] << 8);
    fix->month = message_buf[6];
    fix->day = message_buf[7];
    fix->hour = message_buf[8];
    fix->min = message_buf[9];
    fix->sec = message_buf[10];
    fix->date_valid = message_buf[11] & 0x1;
    fix->time_valid = (message_buf[11] & 0x2) >> 1;
    fix->time_resolved = (message_buf[11] & 0x4) >> 2;
    fix->fix_type = message_buf[20];
    fix->fix_valid = message_buf[21] & 0x1;
    fix->diff_used = (message_buf[21] & 0x2) >> 1;
    fix->psm_state = (message_buf[21] & 0x1C) >> 2;
    fix->hdg_veh_valid = (message_buf[21] & 0x20) >> 5;
    fix->carrier_phase = (message_buf[21] & 0xC0) >> 6;
    fix->num_sats = message_buf[23];
    fix->lon =
        (float)((int32_t)(message_buf[24] + (message_buf[25] << 8) +
                          (message_buf[26] << 16) + (message_buf[27] << 24))) /
        10000000.0;
    fix->lat =
        (float)((int32_t)(message_buf[28] + (message_buf[29] << 8) +
                          (message_buf[30] << 16) + (message_buf[31] << 24))) /
        10000000.0;
    fix->height =
        (float)((int32_t)(message_buf[32] + (message_buf[33] << 8) +
                          (message_buf[34] << 16) + (message_buf[35] << 24))) /
        1000.0;
    fix->height_msl =
        (float)((int32_t)(message_buf[36] + (message_buf[37] << 8) +
                          (message_buf[38] << 16) + (message_buf[39] << 24))) /
        1000.0;
    fix->accuracy_horiz =
        (float)((uint32_t)(message_buf[40] + (message_buf[41] << 8) +
                           (message_buf[42] << 16) + (message_buf[43] << 24))) /
        1000.0;
    fix->accuracy_vertical =
        (float)((uint32_t)(message_buf[44] + (message_buf[45] << 8) +
                           (message_buf[46] << 16) + (message_buf[47] << 24))) /
        1000.0;
    fix->vel_north =
        (float)((int32_t)(message_buf[48] + (message_buf[49] << 8) +
                          (message_buf[50] << 16) + (message_buf[51] << 24))) /
        1000.0;
    fix->vel_east =
        (float)((int32_t)(message_buf[52] + (message_buf[53] << 8) +
                          (message_buf[54] << 16) + (message_buf[55] << 24))) /
        1000.0;
    fix->vel_down =
        (float)((int32_t)(message_buf[56] + (message_buf[57] << 8) +
                          (message_buf[58] << 16) + (message_buf[59] << 24))) /
        1000.0;
    fix->ground_speed =
        (float)((int32_t)(message_buf[60] + (message_buf[61] << 8) +
                          (message_buf[62] << 16) + (message_buf[63] << 24))) /
        1000.0;
    fix->hdg =
        (float)((int32_t)(message_buf[64] + (message_buf[65] << 8) +
                          (message_buf[66] << 16) + (message_buf[67] << 24))) /
        100000.0;
    fix->accuracy_speed =
        (float)((int32_t)(message_buf[68] + (message_buf[69] << 8) +
                          (message_buf[70] << 16) + (message_buf[71] << 24))) /
        1000.0;
    fix->accuracy_hdg =
        (float)((int32_t)(message_buf[72] + (message_buf[73] << 8) +
                          (message_buf[74] << 16) + (message_buf[75] << 24))) /
        1000.0;
    fix->invalid_llh = message_buf[78] & 0x1;

    return STATUS_OK;
}