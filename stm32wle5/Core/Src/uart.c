#include "peripherals/uart/uart.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32wlxx_hal.h"

#define DMA_WRITE_PTR ((UART_BUFFER_SIZE - huart1.hdmarx->Instance->CNDTR) & (UART_BUFFER_SIZE - 1))

extern UART_HandleTypeDef huart1;

UART_HandleTypeDef* uart_handles[10] = {NULL, NULL, NULL, NULL, NULL,
                                        NULL, NULL, NULL, NULL, NULL};

UartBuffer* uart_buffers[10] = {NULL, NULL, NULL, NULL, NULL,
                                NULL, NULL, NULL, NULL, NULL};

Status uart_setup(UartDevice* dev) {
    switch (dev->periph) {
        case P_UART1:
            uart_handles[dev->periph] = &huart1;
            uart_buffers[dev->periph] = malloc(sizeof(UartBuffer));
            circular_buffer_init(uart_buffers[dev->periph], UART_BUFFER_SIZE);
            return STATUS_OK;
            break;
        default:
            return STATUS_PARAMETER_ERROR;
            break;
    }
}

Status uart_send(UartDevice* dev, uint8_t* buf, uint16_t len) {
    if (uart_handles[dev->periph] == NULL) {
        if (uart_setup(dev) != STATUS_OK) {
            return STATUS_PARAMETER_ERROR;
        }
    }
    if (HAL_UART_Transmit(uart_handles[dev->periph], buf, len, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status start_uart_reading(UartDevice* dev) {
    if (uart_handles[dev->periph] == NULL) {
        if (uart_setup(dev) != STATUS_OK) {
            return STATUS_PARAMETER_ERROR;
        }
    }
    if (HAL_UART_Receive_DMA(uart_handles[dev->periph],
                             uart_buffers[dev->periph]->buffer,
                             UART_BUFFER_SIZE) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status stop_uart_reading(UartDevice* dev) {
	if (uart_handles[dev->periph] != NULL) {
		if (HAL_UART_AbortReceive(uart_handles[dev->periph]) != HAL_OK) {
			return STATUS_ERROR;
		}
		return STATUS_OK;
	}
	return STATUS_PARAMETER_ERROR;
}

void circular_buffer_init(UartBuffer* cb, size_t capacity) {
    cb->rd_ptr = 0;
}

uint8_t circular_buffer_is_empty(const UartBuffer* cb) {
	if(cb->rd_ptr == DMA_WRITE_PTR) {
		return 1;
	}
	return 0;
}

size_t circular_buffer_size(const UartBuffer* cb) {
    return DMA_WRITE_PTR - cb->rd_ptr;
}

uint8_t circular_buffer_pop(UartBuffer* cb, uint8_t* data) {
	if(cb->rd_ptr != DMA_WRITE_PTR) {
		*data = cb->buffer[cb->rd_ptr++];
		cb->rd_ptr %= UART_BUFFER_SIZE;
		return 1;
	} else {
		return 0;
	}
}

UartBuffer* get_circular_buffer(UartDevice* dev) {
	return uart_buffers[dev->periph];
}
