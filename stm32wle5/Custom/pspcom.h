#ifndef PSPCOM_H
#define PSPCOM

#include <stdint.h>

#include "lib/peripherals/uart/uart.h"
#include "lib/common/status.h"

#define CRC16_POLY 0x1021
#define CRC16_INIT 0xFFFF
#define DEVICE_ID 0x10

enum {
    NACK = 0x00,
    ACK = 0x01,
    GETDEVS = 0x02,
    POLLDEV = 0x03,
    GETMSGS = 0x04,
    SETFREQ = 0x11,
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
    MAG = 0x88,
    TIME = 0x89,
    GPS_POS = 0x8A,
    GPS_VEL = 0x8B,
    SYS_STAT = 0x8C,
};

typedef struct {
    uint8_t payload_len;
    uint8_t device_id;
    uint8_t msg_id;
    uint8_t payload[256];
} pspcommsg;

uint16_t crc(uint16_t checksum, pspcommsg msg);

pspcommsg pspcom_process_bytes_from_air(uint8_t* buf, uint16_t len);

void pspcom_send_msg_over_air(pspcommsg msg);

void pspcom_send_msg_over_uart(UartDevice* dev, pspcommsg msg);

Status pspcom_read_msg_from_uart(UartDevice* dev, pspcommsg *msg);

uint8_t pspcom_uart_available(UartDevice* dev);

#endif
