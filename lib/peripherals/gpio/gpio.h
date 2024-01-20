#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

#include "status.h"

typedef enum {
    GPIO_INPUT = 0,
    GPIO_OUTPUT = 1,
    GPIO_INPUT_PULLUP = 2,
    GPIO_INPUT_PULLDOWN = 3,
    GPIO_OUTPUT_OD = 4,
} GpioMode;

typedef enum {
    GPIO_ERR = -1,
    GPIO_LOW = 0,
    GPIO_HIGH = 1,
} GpioValue;

Status gpio_mode(uint8_t pin, GpioMode mode);

Status gpio_write(uint8_t pin, GpioValue value);

GpioValue gpio_read(uint8_t pin);

#endif // GPIO_H
