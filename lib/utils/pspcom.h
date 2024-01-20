#ifndef PSPCOM_H
#define PSPCOM

#include <stdint.h>

#include "sensor.pb.h"
#include "status.h"

#define CRC16_POLY 0x1021
#define CRC16_INIT 0xFFFF

enum {
    MAINSTAT = 0x81,
    DRGSTAT = 0x82,
    AUXSTAT = 0x83,
    ACCEL = 0x84,
    GYRO = 0x85,
    TEMP = 0x86,
    PRES = 0x87,
};

typedef struct {
    uint8_t payload_len;
    uint8_t device_id;
    uint8_t msg_id;
    uint8_t payload[256];
} pspcommsg;

uint16_t crc(uint16_t checksum, pspcommsg msg);

Status pspcom_init();

void pspcom_process_bytes(char *buf, int len);

void pspcom_send_msg(pspcommsg msg);

void pspcom_send_sensor(SensorFrame *sens);

#endif