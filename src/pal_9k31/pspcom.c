#include "pspcom.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define PSPCOM_DEVICE_ID 1

uint16_t crc(uint16_t checksum, pspcommsg msg) {
    // Some code is taken from ChatGPT
    uint8_t *msg_content = (uint8_t *)malloc(5 + msg.payload_len);
    msg_content[0] = '!';
    msg_content[1] = '$';
    msg_content[2] = (uint8_t)msg.payload_len;
    msg_content[3] = (uint8_t)msg.device_id;
    msg_content[4] = (uint8_t)msg.msg_id;
    for (int i = 0; i < msg.payload_len; i++) {
        msg_content[5 + i] = msg.payload[i];
    }
    for (int i = 0; i < 5 + msg.payload_len; i++) {
        checksum ^= (uint16_t)msg_content[i] << 8;  // XOR with next byte
        for (int j = 0; j < 8; j++) {
            if (checksum & 0x8000) {
                checksum = (checksum << 1) ^ CRC16_POLY;
            } else {
                checksum <<= 1;
            }
        }
    }
    free(msg_content);
    return checksum;
}

void pspcom_process_bytes(char *buf, int len) {}

void pspcom_send_msg(pspcommsg msg) {
    uint16_t checksum = crc(CRC16_INIT, msg);
    printf("!$%c%c%c", msg.payload_len, msg.device_id, msg.msg_id);
    fwrite(msg.payload, 1, msg.payload_len, stdout);
    printf("%c%c", (uint8_t)checksum, (uint8_t)(checksum >> 8));
    fflush(stdout);
}

void pspcom_send_sensor(SensorData *sens) {
    // Accelerometer
    pspcommsg tx_msg = {
        .payload_len = 13,
        .device_id = PSPCOM_DEVICE_ID,
        .msg_id = ACCEL,
    };
    tx_msg.payload[0] = 0;
    memcpy(tx_msg.payload + 1, &sens->accel.accelX, 4);
    memcpy(tx_msg.payload + 5, &sens->accel.accelY, 4);
    memcpy(tx_msg.payload + 9, &sens->accel.accelZ, 4);
    pspcom_send_msg(tx_msg);

    // Gyroscope
    tx_msg.msg_id = GYRO;
    memcpy(tx_msg.payload + 1, &sens->gyro.gyroX, 4);
    memcpy(tx_msg.payload + 5, &sens->gyro.gyroY, 4);
    memcpy(tx_msg.payload + 9, &sens->gyro.gyroZ, 4);
    pspcom_send_msg(tx_msg);

    // Temperature
    tx_msg.msg_id = TEMP;
    tx_msg.payload_len = 5;
    memcpy(tx_msg.payload + 1, &sens->baro.temperature, 4);
    pspcom_send_msg(tx_msg);

    // Pressure
    tx_msg.msg_id = PRES;
    memcpy(tx_msg.payload + 1, &sens->baro.pressure, 4);
    pspcom_send_msg(tx_msg);
}