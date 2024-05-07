#include "pspcom.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include "psp_timer.h"

#define PSPCOM_DEVICE_ID 1

extern volatile LoraState lora_state;
extern const struct Radio_s Radio;

void timeout_callback() {
	lora_state = RX;
	Radio.Rx(RX_TIMEOUT_VALUE);
}

uint16_t crc(uint16_t checksum, pspcommsg msg) {
    // Some code is taken from ChatGPT
    uint8_t *msg_content = (uint8_t *)malloc(5 + msg.payload_len);
    msg_content[0] = '!';
    msg_content[1] = '$';
    msg_content[2] = msg.payload_len;
    msg_content[3] = msg.device_id;
    msg_content[4] = msg.msg_id;
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

pspcommsg pspcom_process_bytes_from_air(uint8_t *buf, uint16_t len) {
    pspcommsg result;
    memset(&result, 0, sizeof(pspcommsg));
    if (len < 3) {
        return result;
    }
    result.payload_len = buf[0];
    result.device_id = buf[1];
    result.msg_id = buf[2];
    if (len != 3 + result.payload_len) {
        memset(&result, 0, sizeof(pspcommsg));
        return result;
    }
    memcpy(result.payload, buf + 3, result.payload_len);
    return result;
}

void pspcom_send_msg_over_air(pspcommsg msg) {
    uint8_t payload[256];
    uint8_t len = 2 + msg.payload_len;

    payload[0] = msg.device_id;
    payload[1] = msg.msg_id;
    for (int i = 0; i < msg.payload_len; i++) {
        payload[2 + i] = msg.payload[i];
    }

    lora_state = TX;
    Radio.Send(payload, len);
    DELAY(Radio.TimeOnAir(MODEM_LORA, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, 0, len, 1) + 10);
}

void pspcom_send_msg_over_uart(UartDevice *dev, pspcommsg msg) {
	uint8_t payload[256];
    uint8_t len = 7 + msg.payload_len;
    payload[0] = '!';
    payload[1] = '$';
    payload[2] = msg.payload_len;
    payload[3] = msg.device_id;
    payload[4] = msg.msg_id;
    for (int i = 0; i < msg.payload_len; i++) {
        payload[5 + i] = msg.payload[i];
    }
    uint16_t checksum = crc(CRC16_INIT, msg);
    payload[5 + msg.payload_len] = (uint8_t)(checksum & 0xFF);
    payload[6 + msg.payload_len] = (uint8_t)(checksum >> 8);

    uart_send(dev, payload, len);
}

Status pspcom_read_msg_from_uart(UartDevice *dev, pspcommsg *msg) {
    UartBuffer *cb = get_circular_buffer(dev);
    static uint8_t state = 0;
    uint8_t current_byte = 0;
    static uint16_t checksum;
    static uint8_t payload_ctr = 0;

    if (cb == NULL) {
        return STATUS_ERROR;
    }
    while (!circular_buffer_is_empty(cb)) {
        switch (state) {
            case 0:
            	memset(msg, 0, sizeof(pspcommsg));
            	payload_ctr = 0;
            	checksum = 0;
                circular_buffer_pop(cb, &current_byte);
                if (current_byte == '!') {
                    state = 1;
                }
                break;
            case 1:
                circular_buffer_pop(cb, &current_byte);
                if (current_byte == '$') {
                    state = 2;
                } else {
                    state = 0;
                }
                break;
            case 2:
                circular_buffer_pop(cb, &current_byte);
                msg->payload_len = current_byte;
                state = 3;
                break;
            case 3:
                circular_buffer_pop(cb, &current_byte);
                msg->device_id = current_byte;
                state = 4;
                break;
            case 4:
                circular_buffer_pop(cb, &current_byte);
                msg->msg_id = current_byte;
                state = 5;
                break;
            case 5:
                if (payload_ctr < msg->payload_len) {
                    circular_buffer_pop(cb, &current_byte);
                    msg->payload[payload_ctr] = current_byte;
                    payload_ctr++;
                    break;
                } else {
                    state = 6;
                }
            case 6:
                circular_buffer_pop(cb, &current_byte);
                checksum = current_byte;
                state = 7;
                break;
            case 7:
                circular_buffer_pop(cb, &current_byte);
                checksum += ((uint16_t)current_byte) << 8;
                if (crc(CRC16_INIT, *msg) == checksum) {
                	state = 0;
                    return STATUS_OK;
                }
                state = 0;
                break;
        }
    }
	return STATUS_ERROR;
}

uint8_t pspcom_uart_available(UartDevice *dev) {
    UartBuffer *cb = get_circular_buffer(dev);
    if (cb == NULL) {
        return 0;
    }
    return !circular_buffer_is_empty(cb);
}
