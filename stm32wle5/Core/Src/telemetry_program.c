#include "telemetry_program.h"

#include "subghz_phy_app.h"
#include "peripherals/uart/uart.h"
#include "pspcom.h"
#include "radio.h"
#include "stdio.h"

extern volatile LoraState lora_state;
extern const struct Radio_s Radio;

UartDevice uart1 = {
    .periph = P_UART1,
    .speed = UART_SPEED_115200,
};

void telemetry_program() {
	start_uart_reading(&uart1);
    while (1) {
        process_packet_from_uart();
    }
}

void process_packet_from_air(uint8_t *payload, uint16_t length) {
    pspcommsg rx_msg = pspcom_process_bytes_from_air(payload, length);
    pspcommsg tx_msg = {0};

    // Message was not addressed to this device
    if (rx_msg.device_id != DEVICE_ID && rx_msg.device_id != 0) {
        return;
    }
    switch (rx_msg.msg_id) {
        // Arm/disarm
        case ARMMAIN:
        case ARMDRG:
        case ARMAUX:
        case DISARMMAIN:
        case DISARMDRG:
        case DISARMAUX:
            rx_msg.device_id = DEVICE_ID;
            pspcom_send_msg_over_uart(&uart1, rx_msg);
            break;

        // Fire
        case FIREMAIN:
        case FIREDRG:
        case FIREAUX:
            // Tell the device to fire
            pspcom_send_msg_over_uart(&uart1, rx_msg);
            tx_msg.msg_id = ACK;
            tx_msg.device_id = DEVICE_ID;
            tx_msg.payload_len = 0;
            // Send ACK
            pspcom_send_msg_over_air(tx_msg);
            break;

        // Get devices
        case GETDEVS:
            tx_msg.msg_id = GETDEVS;
            tx_msg.device_id = DEVICE_ID;
            tx_msg.payload_len = 0;
            pspcom_send_msg_over_air(tx_msg);
            break;

        // Poll device
        case POLLDEV:
            tx_msg.msg_id = ACK;
            tx_msg.device_id = DEVICE_ID;
            tx_msg.payload_len = 0;
            pspcom_send_msg_over_air(tx_msg);
            break;

        // Get messages
        case GETMSGS:
            tx_msg.msg_id = GETMSGS;
            tx_msg.device_id = DEVICE_ID;
            tx_msg.payload_len = 0;
            pspcom_send_msg_over_air(tx_msg);
            break;

        // Set frequency
        case SETFREQ:
            tx_msg.msg_id = ACK;
            tx_msg.device_id = DEVICE_ID;
            tx_msg.payload_len = 0;
            pspcom_send_msg_over_air(tx_msg);
            while (lora_state == TX)
                ;
            Radio.SetChannel(*((uint32_t *)(rx_msg.payload + 1)));
            Radio.Rx(RX_TIMEOUT_VALUE);
            lora_state = RX;
            break;

        default:
            break;
    }
}

void process_packet_from_uart() {
    if (!pspcom_uart_available(&uart1)) {
        return;
    }
    static pspcommsg rx_msg = {0};
    if(pspcom_read_msg_from_uart(&uart1, &rx_msg) != STATUS_OK) {
    	return;
    }
    if (rx_msg.msg_id == 0) {
        return;
    }
    printf("%d %d %d\n", rx_msg.msg_id, rx_msg.device_id, rx_msg.payload_len);
    switch (rx_msg.msg_id) {
        default:
            rx_msg.device_id = DEVICE_ID;
            pspcom_send_msg_over_air(rx_msg);
            break;
    }
}
