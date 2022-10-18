#include "gpio/gpio.h"

#include "registers.h"
#include "stdlib.h"

static GPIO_t *getGpio(uint8_t pin) {
    GPIO_t *base = NULL;
    uint8_t gpioGroup = PIN[pin].gpio_pin / 100 + 5;
    if (gpioGroup == 6) {
        base = GPIO6;
    } else if (gpioGroup == 7) {
        base = GPIO7;
    } else if (gpioGroup == 8) {
        base = GPIO8;
    } else if (gpioGroup == 9) {
        base = GPIO9;
    }
    return base;
}

Status gpio_mode(uint8_t pin, GpioMode mode) {
    *(PIN[pin].MUX_REG_ADDR) = (*(PIN[pin].MUX_REG_ADDR) & 0xFFFFFFF8) |
                               0x00000005;  // Set to GPIO Mode
    GPIO_t *base = getGpio(pin);
    if (base == NULL) {
        return ERROR;
    }
    uint32_t gpioPin = (uint32_t)PIN[pin].gpio_pin % 100;
    if (mode) {
        base->GDIR |= (1 << gpioPin);
    } else {
        base->GDIR &= ~(1 << gpioPin);
    }
    return OK;
}

Status gpio_write(uint8_t pin, GpioValue value) {
    *(PIN[pin].MUX_REG_ADDR) = (*(PIN[pin].MUX_REG_ADDR) & 0xFFFFFFF8) |
                               0x00000005;  // Set to GPIO Mode
    GPIO_t *base = getGpio(pin);
    if (base == NULL) {
        return ERROR;
    }
    uint32_t gpioPin = (uint32_t)PIN[pin].gpio_pin % 100;
    if (!((base->GDIR) & (1 << gpioPin))) {
        base->GDIR |= (1 << gpioPin);
    }
    if (value) {
        base->DR_SET |= (1 << gpioPin);
    } else {
        base->DR_CLEAR |= (1 << gpioPin);
    }
    return OK;
}

GpioValue gpio_read(uint8_t pin) {
    *(PIN[pin].MUX_REG_ADDR) = (*(PIN[pin].MUX_REG_ADDR) & 0xFFFFFFF8) |
                               0x00000005;  // Set to GPIO Mode
    GPIO_t *base = getGpio(pin);
    if (base == NULL) {
        return GPIO_ERR;
    }
    uint32_t gpioPin = (uint32_t)PIN[pin].gpio_pin % 100;
    if ((base->GDIR) & (1 << gpioPin)) {
        base->GDIR &= ~(1 << gpioPin);
    }
    return ((base->DR) & (1 << gpioPin)) >> gpioPin;
}