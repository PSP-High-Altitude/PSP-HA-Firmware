#include "sx1276.h"

#include "string.h"
#include "timer.h"

bool s_sx1276_initialized = false;
bool s_implicit_header = false;

static Status sx1276_read(SpiDevice* device, uint8_t address, uint8_t* rx_buf,
                          uint8_t len) {
    if (address > 127) {
        return STATUS_PARAMETER_ERROR;
    }

    // Create tx buffer with extra dummy bytes
    uint8_t tx_buf[len + 1];
    // Create rx buffer with space for the initial blank byte
    uint8_t rx_buf_new[len + 1];

    tx_buf[0] = address;  // Add address and read bit to tx buffer

    // Exchange the address and read len bits then copy all but the first byte
    // received to the original rx_buf.
    Status status = spi_exchange(device, tx_buf, rx_buf_new, len + 1);
    memcpy(rx_buf, rx_buf_new + 1, len);

    // DELAY_MICROS(100);

    return status;
}

static Status sx1276_write(SpiDevice* device, uint8_t address, uint8_t* tx_buf,
                           uint8_t len) {
    if (address > 127) {
        return STATUS_PARAMETER_ERROR;
    }

    // Create dummy read buffer
    uint8_t rx_buf[len + 1];

    // Create new tx buffer
    uint8_t tx_buf_new[len + 1];
    tx_buf_new[0] = 0x80 | address;  // Add address and write bit to tx buffer
    memcpy(tx_buf_new + 1, tx_buf,
           len);  // Copy bytes to end of the new tx buffer

    // Exchange address and write len bytes
    Status status = spi_exchange(device, tx_buf_new, rx_buf, len + 1);

    // DELAY_MICROS(100);

    return status;
}

Status sx1276_init(SpiDevice* spi_device, int reset_pin, int freq_hz,
                   int power_dbm, int bandwidth_hz, int spreading_factor,
                   int coding_rate, int preamble_len, bool implicit_header,
                   bool crc_on, bool low_data_rate) {
    uint8_t tx_buf = 0;
    uint8_t rx_buf = 0;

    // Reset
    gpio_write(reset_pin, GPIO_LOW);
    DELAY_MICROS(110);
    gpio_write(reset_pin, GPIO_HIGH);
    DELAY(5);

    // Check version
    if (sx1276_read(spi_device, SX1276_REG_VERSION, &rx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (rx_buf != 0x12) {
        return STATUS_ERROR;
    }

    // Go to sleep
    tx_buf = 0x04;
    if (sx1276_write(spi_device, SX1276_REG_OP_MODE, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Set LoRa mode, low frequency mode
    tx_buf = 0x88;
    if (sx1276_write(spi_device, SX1276_REG_OP_MODE, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Set frequency
    uint64_t freq = ((uint64_t)freq_hz << 19) / 32000000;
    if (freq & 0xFF000000) {
        return STATUS_PARAMETER_ERROR;
    }
    tx_buf = (freq >> 16) & 0xFF;
    if (sx1276_write(spi_device, SX1276_REG_FRF_MSB, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    tx_buf = (freq >> 8) & 0xFF;
    if (sx1276_write(spi_device, SX1276_REG_FRF_MID, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    tx_buf = freq & 0xFF;
    if (sx1276_write(spi_device, SX1276_REG_FRF_LSB, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Set power
    if (power_dbm > 20) {
        return STATUS_PARAMETER_ERROR;
    }
    if (power_dbm > 17) {
        tx_buf = 0xFF;
    } else if (power_dbm > 14) {
        tx_buf = 0xC0 | (power_dbm - 2);
    } else {
        tx_buf = 0xD0 | (power_dbm + 1);
    }
    if (sx1276_write(spi_device, SX1276_REG_PA_CONFIG, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }
    tx_buf = (power_dbm > 17) ? 0x87 : 0x84;
    if (sx1276_write(spi_device, SX1276_REG_PA_DAC, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Modem config 1
    int bw_reg = -1;

    // Bandwidth
    int bandwidth_arr[] = {7800,  10400, 15600,  20800,  31250,
                           41700, 62500, 125000, 250000, 500000};
    for (int i = 0; i < (sizeof(bandwidth_arr) / sizeof(bandwidth_arr[0]));
         i++) {
        if (bandwidth_hz == bandwidth_arr[i]) {
            bw_reg = i;
            break;
        }
    }
    if (bw_reg == -1) {
        return STATUS_PARAMETER_ERROR;
    }

    // Coding rate
    if (coding_rate < 5 || coding_rate > 8) {
        return STATUS_PARAMETER_ERROR;
    }
    int cr_reg = coding_rate - 4;

    // Implicit header
    s_implicit_header = implicit_header;
    int ih_reg = implicit_header ? 1 : 0;

    // Modem config 1
    tx_buf = (bw_reg << 4) | (cr_reg << 1) | ih_reg;
    if (sx1276_write(spi_device, SX1276_REG_MODEM_CONFIG1, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Modem config 2

    // Spreading factor
    int sf_reg = spreading_factor;
    if (sf_reg < 6 || sf_reg > 12) {
        return STATUS_PARAMETER_ERROR;
    }

    // CRC
    int crc_reg = crc_on ? 1 : 0;

    // Modem config 2
    tx_buf = (sf_reg << 4) | (crc_reg << 2);
    if (sx1276_write(spi_device, SX1276_REG_MODEM_CONFIG2, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Preamble length
    if (preamble_len > 0xFFFF) {
        return STATUS_PARAMETER_ERROR;
    }
    tx_buf = (preamble_len >> 8) & 0xFF;
    if (sx1276_write(spi_device, SX1276_REG_PREAMBLE_MSB, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }
    tx_buf = preamble_len & 0xFF;
    if (sx1276_write(spi_device, SX1276_REG_PREAMBLE_LSB, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Low data rate
    int ldr_reg = low_data_rate ? 1 : 0;
    tx_buf = ldr_reg << 3;
    if (sx1276_write(spi_device, SX1276_REG_MODEM_CONFIG3, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Detection optimize
    if (spreading_factor > 6) {
        tx_buf = 0xC3;
    } else {
        tx_buf = 0xC5;
    }
    if (sx1276_write(spi_device, SX1276_REG_DETECT_OPTIMIZE, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Detection threshold
    if (spreading_factor > 6) {
        tx_buf = 0x0A;
    } else {
        tx_buf = 0x0C;
    }
    if (sx1276_write(spi_device, SX1276_REG_DETECTION_THRESHOLD, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // 500kHz bandwidth optimize
    if (bandwidth_hz == 500000) {
        tx_buf = 0x64;
        if (sx1276_write(spi_device, SX1276_REG_HIGH_BW_OPTIMIZE1, &tx_buf,
                         1) != STATUS_OK) {
            return STATUS_ERROR;
        }
    }

    // TCXO config
    tx_buf = 0x19;
    if (sx1276_write(spi_device, SX1276_REG_TCXO, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Standby mode
    tx_buf = 0x89;
    if (sx1276_write(spi_device, SX1276_REG_OP_MODE, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    s_sx1276_initialized = true;

    return STATUS_OK;
}

Status sx1276_set_rx_payload_length(SpiDevice* spi_device, int len) {
    if (!s_sx1276_initialized) {
        return STATUS_ERROR;
    }

    // Set expected payload length (for implicit header
    uint8_t tx_buf = len;
    if (sx1276_write(spi_device, SX1276_REG_PAYLOAD_LENGTH, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

Status sx1276_transmit(SpiDevice* spi_device, uint8_t* data, int len) {
    if (!s_sx1276_initialized) {
        return STATUS_ERROR;
    }

    // Set payload length
    uint8_t tx_buf = len;
    if (sx1276_write(spi_device, SX1276_REG_PAYLOAD_LENGTH, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Read TX FIFO base address
    uint8_t tx_fifo_base;
    if (sx1276_read(spi_device, SX1276_REG_FIFO_TX_BASE_ADDR, &tx_fifo_base,
                    1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Set FIFO address pointer to tx FIFO base address
    if (sx1276_write(spi_device, SX1276_REG_FIFO_ADDR_PTR, &tx_fifo_base, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Write data to FIFO
    if (sx1276_write(spi_device, SX1276_REG_FIFO, data, len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Start TX
    tx_buf = 0x8B;
    if (sx1276_write(spi_device, SX1276_REG_OP_MODE, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    while (1) {
        // Read IRQ flags
        uint8_t irq_flags;
        if (sx1276_read(spi_device, SX1276_REG_IRQ_FLAGS, &irq_flags, 1) !=
            STATUS_OK) {
            return STATUS_ERROR;
        }

        // Check if TX done
        if (irq_flags & 0x08) {
            // Clear TX done flag
            tx_buf = 0x08;
            if (sx1276_write(spi_device, SX1276_REG_IRQ_FLAGS, &tx_buf, 1) !=
                STATUS_OK) {
                return STATUS_ERROR;
            }

            return STATUS_OK;
        }

        // Yielding Delay
        DELAY(1);
    }

    return STATUS_ERROR;
}

Status sx1276_start_receive(SpiDevice* spi_device) {
    if (!s_sx1276_initialized) {
        return STATUS_ERROR;
    }

    // Read current mode
    uint8_t rx_buf = 0xFF;

    // Wait for standby/sleep
    while ((rx_buf & 0x7) > 1) {
        // Read mode
        if (sx1276_read(spi_device, SX1276_REG_OP_MODE, &rx_buf, 1) !=
            STATUS_OK) {
            return STATUS_ERROR;
        }

        // Yielding Delay
        DELAY(1);
    }

    // Set RX continuous mode
    uint8_t tx_buf = 0x8D;
    if (sx1276_write(spi_device, SX1276_REG_OP_MODE, &tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

int sx1276_packet_available(SpiDevice* spi_device) {
    if (!s_sx1276_initialized) {
        return -1;
    }

    // Read IRQ flags
    uint8_t irq_flags;
    if (sx1276_read(spi_device, SX1276_REG_IRQ_FLAGS, &irq_flags, 1) !=
        STATUS_OK) {
        return -1;
    }

    // Check if RX done
    if (irq_flags & 0x40) {
        return 1;
    }

    return 0;
}

Status sx1276_read_packet(SpiDevice* spi_device, uint8_t* data, int* len) {
    if (!s_sx1276_initialized) {
        return STATUS_ERROR;
    }

    // Read RX FIFO address
    uint8_t rx_buf;
    if (sx1276_read(spi_device, SX1276_REG_FIFO_RX_CURRENT_ADDR, &rx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Set FIFO address pointer to RX FIFO base address
    if (sx1276_write(spi_device, SX1276_REG_FIFO_ADDR_PTR, &rx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Read packet length
    if (sx1276_read(spi_device, SX1276_REG_RX_NB_BYTES, (uint8_t*)len, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    // Read data
    if (sx1276_read(spi_device, SX1276_REG_FIFO, data, *len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Clear RX done flag
    uint8_t tx_buf = 0x40;
    if (sx1276_write(spi_device, SX1276_REG_IRQ_FLAGS, &tx_buf, 1) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}