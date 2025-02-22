#include "uart/uart.h"

#include "gpio/gpio.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// TX ONLY IMPLEMENTATION
static void uart_bitbang_packet(UartDevice* device, uint8_t data) {
    float bit_period_us = 1e6 / device->baudrate;
    float next_bit_transmit_time = 0.;
    uint64_t start_us = MICROS();
    uint64_t tgt_time_us;
    int64_t delay_us;

    // Start bit
    gpio_write(device->tx, GPIO_LOW);
    next_bit_transmit_time += bit_period_us;

    tgt_time_us = start_us + (uint64_t)next_bit_transmit_time;
    delay_us = tgt_time_us - MICROS();
    if (delay_us > 0) {
        DELAY_MICROS(delay_us);
    }

    // Loop over the 8 data bits
    for (int i = 0; i < 8; i++) {
        gpio_write(device->tx, (data >> i) & 1);
        next_bit_transmit_time += bit_period_us;

        tgt_time_us = start_us + (uint64_t)next_bit_transmit_time;
        delay_us = tgt_time_us - MICROS();
        if (delay_us > 0) {
            DELAY_MICROS(delay_us);
        }
    }

    // Stop bit
    gpio_write(device->tx, GPIO_HIGH);
    next_bit_transmit_time += bit_period_us;

    tgt_time_us = start_us + (uint64_t)next_bit_transmit_time;
    delay_us = tgt_time_us - MICROS();
    if (delay_us > 0) {
        DELAY_MICROS(delay_us);
    }
}

Status uart_init(UartDevice* device) {
    gpio_mode(device->tx, GPIO_OUTPUT);
    gpio_write(device->tx, GPIO_HIGH);

    return STATUS_OK;
}

Status uart_tx(UartDevice* device, uint8_t* tx_buf, size_t len) {
    taskENTER_CRITICAL();

    for (int i = 0; i < len; i++) {
        uart_bitbang_packet(device, tx_buf[i]);
    }

    taskEXIT_CRITICAL();

    return STATUS_OK;
}

Status uart_rx(UartDevice* device, uint8_t* rx_buf, size_t len) {
    return STATUS_ERROR;
}
