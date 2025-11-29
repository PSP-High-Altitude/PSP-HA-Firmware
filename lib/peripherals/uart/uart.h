#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdlib.h>

#include "status.h"

typedef struct {
    uint32_t baudrate;
    uint16_t tx;  // Transmit pin
    uint16_t rx;  // Receive pin
} UartDevice;

Status uart_init(UartDevice* device);

Status uart_tx(UartDevice* device, uint8_t* tx_buf, size_t len);

Status uart_rx(UartDevice* device, uint8_t* rx_buf, size_t len);

#endif // UART_H
