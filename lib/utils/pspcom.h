#ifndef PSPCOM_H
#define PSPCOM

#include <stdint.h>

#include "flight_control.h"
#include "max_m10s.h"
#include "sensor.pb.h"
#include "status.h"

#define CRC16_POLY 0x1021
#define CRC16_INIT 0xFFFF

enum {
    NACK = 0x00,
    ACK = 0x01,
    SET_LOCAL_FREQ = 0x5,
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
    STD_TELEM_1 = 0xE0,
    STD_TELEM_2 = 0xE1,
};

typedef struct {
    uint8_t payload_len;
    uint8_t device_id;
    uint8_t msg_id;
    uint8_t payload[256];
} pspcommsg;

typedef struct __attribute__((packed)) {
    uint8_t num_sats : 5;
    int32_t lat : 24;
    int32_t lon : 25;
    uint32_t alt : 18;
} gps_pos_packed;

typedef struct __attribute__((packed)) {
    int32_t veln : 13;
    int32_t vele : 13;
    int32_t veld : 14;
} gps_vel_packed;

typedef struct {
    SensorFrame *sensor_frame;
    GPS_Fix_TypeDef *gps_fix;
} PAL_Data_Typedef;

uint16_t crc(uint16_t checksum, pspcommsg msg);

Status pspcom_init();

void task_pspcom_rx();

void pspcom_send_msg(pspcommsg msg);

void pspcom_send_sensor(SensorFrame* sensor_frame);

void pspcom_send_gps(GPS_Fix_TypeDef* gps_fix);

void pspcom_update_sensors(SensorFrame* sensor_frame);

void pspcom_update_gps(GPS_Fix_TypeDef* gps_fix);

void pspcom_update_fp(FlightPhase flight_phase);

void task_pspcom_tx();

void pspcom_send_status();

void pspcom_send_standard();

Status pspcom_change_frequency(uint32_t frequency_hz);

#endif