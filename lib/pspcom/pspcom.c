#include "pspcom.h"

#include <stdlib.h>

#include "board_mgmt.h"
#include "camera.h"
#include "pyros.h"
#include "timer.h"

#define PSPCOM_DEVICE_ID 0x10  // PAL 9k5/Darkstar

static uint8_t pyro_armed[5];
static uint64_t pyro_armed_start[5];

uint16_t crc16(uint16_t checksum, pspcommsg *msg) {
    // Some code is taken from ChatGPT
    uint8_t *msg_content = (uint8_t *)malloc(5 + msg->payload_len);
    msg_content[0] = '!';
    msg_content[1] = '$';
    msg_content[2] = (uint8_t)msg->payload_len;
    msg_content[3] = (uint8_t)msg->device_id;
    msg_content[4] = (uint8_t)msg->msg_id;
    for (int i = 0; i < msg->payload_len; i++) {
        msg_content[5 + i] = msg->payload[i];
    }
    for (int i = 0; i < 5 + msg->payload_len; i++) {
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

Status pspcom_handle_message(pspcommsg *msg) {
    // If the message id is greater than 0x80, the message is
    // telemetry from another board, and we should ignore it
    if (msg->msg_id >= 0x80) {
        return STATUS_OK;
    }

    PAL_LOGI("Received message: %d %d %d\n", msg->device_id, msg->msg_id,
             msg->payload_len);

    // Process message
    switch (msg->msg_id) {
        case ARMMAIN:
            // Arm main pyro
            pyro_armed[0] = 1;
            pyro_armed_start[0] = MILLIS();
            PAL_LOGI("Main pyro armed!\n");
            break;
        case ARMDRG:
            // Arm drogue pyro
            pyro_armed[1] = 1;
            pyro_armed_start[1] = MILLIS();
            PAL_LOGI("Drogue pyro armed!\n");
            break;
        case ARMAUX:
            if (msg->payload_len == 1 && msg->payload[0] < 3) {
                // Arm specified aux pyro
                pyro_armed[msg->payload[0]] = 1;
                pyro_armed_start[msg->payload[0]] = MILLIS();
                PAL_LOGI("A%1d pyro armed!\n", msg->payload[0] + 1);
            } else {
                // Arm the first aux pyro
                pyro_armed[2] = 1;
                pyro_armed_start[2] = MILLIS();
                PAL_LOGI("A1 pyro armed!\n");
            }
            break;
        case FIREMAIN:
            // Fire main pyro (if not timed out)
            if (MILLIS() - pyro_armed_start[0] < ARM_TIMEOUT_MS) {
                pyros_fire(PYRO_MAIN);
            }
            break;
        case FIREDRG:
            // Fire drogue pyro (if not timed out)
            if (MILLIS() - pyro_armed_start[1] < ARM_TIMEOUT_MS) {
                pyros_fire(PYRO_DRG);
            }
            break;
        case FIREAUX:
            if (msg->payload_len == 1 && msg->payload[0] < 3) {
                // Fire specified aux pyro (if not timed out)
                if (MILLIS() - pyro_armed_start[msg->payload[0]] <
                    ARM_TIMEOUT_MS) {
                    pyros_fire(PYRO_A1 + msg->payload[0]);
                }
            } else {
                // Fire the first aux pyro (if not timed out)
                if (MILLIS() - pyro_armed_start[2] < ARM_TIMEOUT_MS) {
                    pyros_fire(PYRO_A1);
                }
            }
            break;
        case RESETDEVICE:
            // Reset board if magic byte received
            if (msg->payload_len == 1 && msg->payload[0] == 0x42) {
                PAL_LOGI("Received hard reset command\n");
                mgmt_hard_reset();
            }
            break;
        case TRIGGERCAM:
            // Send the camera start recording command
            if (msg->payload_len == 1 && msg->payload[0] == 42) {
                if (camera_get_mode() == CAMERA_MODE_OFF) {
                    PAL_LOGI("Starting camera\n");
                    camera_set_mode(CAMERA_MODE_RUN);
                } else if (camera_get_mode() == CAMERA_MODE_RUN) {
                    PAL_LOGI("Stopping camera\n");
                    camera_set_mode(CAMERA_MODE_OFF);
                } else {
                    PAL_LOGE("Camera state is unknown; ignoring trigger\n");
                }
            }
            break;
    }

    return STATUS_OK;
}

pspcommsg pspcom_make_standard(SensorFrame *sensor_frame,
                               GPS_Fix_TypeDef *gps_fix,
                               FlightPhase flight_phase) {
    // Standard telemetry
    pspcommsg msg = {
        .payload_len = 19,
        .device_id = PSPCOM_DEVICE_ID,
        .msg_id = STD_TELEM_2,
    };

    // GPS_POS
    gps_pos_packed gps_pos;
    gps_pos.num_sats = gps_fix->num_sats & 0x1F;
    gps_pos.lat = ((int32_t)(gps_fix->lat / 0.0000108)) & 0x00FFFFFF;
    gps_pos.lon = ((int32_t)(gps_fix->lon / 0.0000108)) & 0x01FFFFFF;
    gps_pos.alt = ((uint32_t)(gps_fix->height_msl + 1000)) & 0x0003FFFF;
    memcpy(msg.payload, &gps_pos, sizeof(gps_pos_packed));

    // GPS_VEL
    gps_vel_packed gps_vel;
    gps_vel.veln = ((int16_t)(gps_fix->vel_north)) & 0x1FFF;
    gps_vel.vele = ((int16_t)(gps_fix->vel_east)) & 0x1FFF;
    gps_vel.veld = ((int16_t)(gps_fix->vel_down)) & 0x3FFF;
    memcpy(msg.payload + 9, &gps_vel, sizeof(gps_vel_packed));

    // PRES
    uint16_t pres = (uint16_t)(sensor_frame->pressure / 0.025);
    msg.payload[14] = pres & 0xFF;
    msg.payload[15] = (pres >> 8) & 0xFF;

    // PYRO_STAT
    uint8_t main_cont = pyros_cont(PYRO_MAIN);
    uint8_t drg_cont = pyros_cont(PYRO_DRG);
    uint8_t a1_cont = pyros_cont(PYRO_A1);
    uint8_t a2_cont = pyros_cont(PYRO_A2);
    uint8_t a3_cont = pyros_cont(PYRO_A3);
    // Show continuity and consider all armed
    msg.payload[16] = (main_cont << 1) | (drg_cont << 3) | (a1_cont << 5) |
                      (a2_cont << 7) | 0x55;
    msg.payload[17] = (a3_cont << 1) | 0x1;

    // SYS_STAT
    uint8_t camera_status = camera_get_mode() == CAMERA_MODE_RUN;
    msg.payload[18] = (gps_fix->fix_valid && !gps_fix->invalid_llh) & 0x1;
    // msg.payload[18] |= ((uint8_t)storage_status() & 0x1) << 1;
    msg.payload[18] |= ((uint8_t)camera_status & 0x1) << 2;
    msg.payload[18] |= ((uint8_t)flight_phase & 0xF) << 3;

    return msg;
}
