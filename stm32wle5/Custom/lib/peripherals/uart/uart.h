#ifndef UART_H
#define UART_H

#include <stdint.h>

#include "common/status.h"
#include "stm32wlxx_hal.h"

#define UART_BUFFER_SIZE 512

typedef struct {
    uint8_t buffer[UART_BUFFER_SIZE];
    size_t rd_ptr;
} UartBuffer;

typedef enum {
    UART_SPEED_9600 = 9600,
    UART_SPEED_19200 = 19200,
    UART_SPEED_38400 = 38400,
    UART_SPEED_57600 = 57600,
    UART_SPEED_115200 = 115200,
} UartSpeed;

typedef enum {
    P_UART1 = 0,
    P_UART2 = 1,
    P_UART3 = 2,
    P_UART4 = 3,
    P_UART5 = 4,
    P_UART6 = 5,
    P_UART7 = 6,
    P_UART8 = 7,
    P_UART9 = 8,
    P_UART10 = 9,
} UartPeriph;

typedef struct {
    UartPeriph periph;
    UartSpeed speed;
} UartDevice;

Status uart_setup(UartDevice* dev);

Status uart_send(UartDevice* dev, uint8_t* buf, uint16_t len);

Status start_uart_reading(UartDevice* dev);

Status stop_uart_reading(UartDevice* dev);

void circular_buffer_init(UartBuffer* cb, size_t capacity);

uint8_t circular_buffer_is_empty(const UartBuffer* cb);

size_t circular_buffer_size(const UartBuffer* cb);

uint8_t circular_buffer_pop(UartBuffer* cb, uint8_t* data);

UartBuffer* get_circular_buffer(UartDevice* dev);

#endif  // UART_H
