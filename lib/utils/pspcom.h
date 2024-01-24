#ifndef PSPCOM_H
#define PSPCOM

#include <stdint.h>

#include "max_m10s.h"
#include "sensor.pb.h"
#include "status.h"

#define CRC16_POLY 0x1021
#define CRC16_INIT 0xFFFF

enum {
    ARMMAIN = 0x12,
    ARMDRG = 0x13,
    ARMAUX = 0x14,
    DISARMMAIN = 0x15,
    DISARMDRG = 0x16,
    DISARMAUX = 0x17,
    FIREMAIN = 0x18,
    FIREDRG = 0x19,
    FIREAUX = 0x1A,
    MAINSTAT = 0x81,
    DRGSTAT = 0x82,
    AUXSTAT = 0x83,
    ACCEL = 0x84,
    GYRO = 0x85,
    TEMP = 0x86,
    PRES = 0x87,
    GPS_POS = 0x8A,
    GPS_VEL = 0x8B,
    SYS_STAT = 0x8C,
    PYRO_STAT = 0x8D,
};

typedef struct {
    uint8_t payload_len;
    uint8_t device_id;
    uint8_t msg_id;
    uint8_t payload[256];
} pspcommsg;

uint16_t crc(uint16_t checksum, pspcommsg msg);

Status pspcom_init();

void pspcom_process_bytes();

void pspcom_send_msg(pspcommsg msg);

void pspcom_send_sensor(void *sensor_frame);

void pspcom_send_gps(void *gps_fix);

void pspcom_send_status();

#endif