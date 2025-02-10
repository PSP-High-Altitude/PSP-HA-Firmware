#ifndef SX1276_H
#define SX1276_H

#include "spi/spi.h"
#include "status.h"

#define SX1276_REG_FIFO 0x00
#define SX1276_REG_OP_MODE 0x01
#define SX1276_REG_FRF_MSB 0x06
#define SX1276_REG_FRF_MID 0x07
#define SX1276_REG_FRF_LSB 0x08
#define SX1276_REG_PA_CONFIG 0x09
#define SX1276_REG_PA_RAMP 0x0A
#define SX1276_REG_OCP 0x0B
#define SX1276_REG_LNA 0x0C
#define SX1276_REG_FIFO_ADDR_PTR 0x0D
#define SX1276_REG_FIFO_TX_BASE_ADDR 0x0E
#define SX1276_REG_FIFO_RX_BASE_ADDR 0x0F
#define SX1276_REG_FIFO_RX_CURRENT_ADDR 0x10
#define SX1276_REG_IRQ_FLAGS_MASK 0x11
#define SX1276_REG_IRQ_FLAGS 0x12
#define SX1276_REG_RX_NB_BYTES 0x13
#define SX1276_REG_RX_HEADER_CNT_VALUE_MSB 0x14
#define SX1276_REG_RX_HEADER_CNT_VALUE_LSB 0x15
#define SX1276_REG_RX_PACKET_CNT_VALUE_MSB 0x16
#define SX1276_REG_RX_PACKET_CNT_VALUE_LSB 0x17
#define SX1276_REG_MODEM_STAT 0x18
#define SX1276_REG_PKT_SNR_VALUE 0x19
#define SX1276_REG_PKT_RSSI_VALUE 0x1A
#define SX1276_REG_RSSI_VALUE 0x1B
#define SX1276_REG_HOP_CHANNEL 0x1C
#define SX1276_REG_MODEM_CONFIG1 0x1D
#define SX1276_REG_MODEM_CONFIG2 0x1E
#define SX1276_REG_SYMB_TIMEOUT_LSB 0x1F
#define SX1276_REG_PREAMBLE_MSB 0x20
#define SX1276_REG_PREAMBLE_LSB 0x21
#define SX1276_REG_PAYLOAD_LENGTH 0x22
#define SX1276_REG_MAX_PAYLOAD_LENGTH 0x23
#define SX1276_REG_HOP_PERIOD 0x24
#define SX1276_REG_FIFO_RX_BYTE_ADDR 0x25
#define SX1276_REG_MODEM_CONFIG3 0x26
#define SX1276_REG_FEI_MSB 0x28
#define SX1276_REG_FEI_MID 0x29
#define SX1276_REG_FEI_LSB 0x2A
#define SX1276_REG_RSSI_WIDEBAND 0x2C
#define SX1276_REG_DETECT_OPTIMIZE 0x31
#define SX1276_REG_INVERT_IQ 0x33
#define SX1276_REG_HIGH_BW_OPTIMIZE1 0x36
#define SX1276_REG_DETECTION_THRESHOLD 0x37
#define SX1276_REG_SYNC_WORD 0x39
#define SX1276_REG_HIGH_BW_OPTIMIZE2 0x3A
#define SX1276_REG_INVERT_IQ2 0x3B
#define SX1276_REG_DIO_MAPPING1 0x40
#define SX1276_REG_DIO_MAPPING2 0x41
#define SX1276_REG_VERSION 0x42
#define SX1276_REG_TCXO 0x4B
#define SX1276_REG_PA_DAC 0x4D
#define SX1276_REG_FORMER_TEMP 0x5B
#define SX1276_REG_AGC_REF 0x61
#define SX1276_REG_AGC_THRESH1 0x62
#define SX1276_REG_AGC_THRESH2 0x63
#define SX1276_REG_AGC_THRESH3 0x64

Status sx1276_init(SpiDevice *spi_device, int reset_pin, int freq_hz,
                   int power_dbm, int bandwidth_hz, int spreading_factor,
                   int coding_rate, int preamble_len, bool implicit_header,
                   bool crc_on, bool low_data_rate);

Status sx1276_transmit(SpiDevice *spi_device, uint8_t *data, int len);
Status sx1276_set_rx_payload_length(SpiDevice *spi_device, int len);
Status sx1276_start_receive(SpiDevice *spi_device);
int sx1276_packet_available(SpiDevice *spi_device);
Status sx1276_read_packet(SpiDevice *spi_device, uint8_t *data, int *len);

#endif  // SX1276_H
