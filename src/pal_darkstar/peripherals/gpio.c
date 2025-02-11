#include "gpio/gpio.h"

#include "board.h"
#include "stm32h7xx_hal.h"

Status gpio_mode(uint8_t pin, GpioMode mode) {
    uint32_t gpio_pin = PAL_GPIO_PIN(pin);
    GPIO_TypeDef *base = PAL_GPIO_PORT(pin);
    GPIO_InitTypeDef conf = {
        .Mode = GPIO_MODE_INPUT,
        .Pin = gpio_pin,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    if (GPIO_INPUT | GPIO_INPUT_PULLDOWN | GPIO_INPUT_PULLUP)
        conf.Mode = GPIO_MODE_INPUT;
    else if (GPIO_OUTPUT)
        conf.Mode = GPIO_MODE_OUTPUT_OD;
    else if (GPIO_OUTPUT_OD)
        conf.Mode = GPIO_MODE_OUTPUT_OD;
    else if (GPIO_ANALOG)
        conf.Mode = GPIO_MODE_ANALOG;
    if (mode == GPIO_INPUT_PULLUP)
        conf.Pull = GPIO_PULLUP;
    else if (mode == GPIO_INPUT_PULLDOWN)
        conf.Pull = GPIO_PULLDOWN;
    else
        conf.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(base, &conf);
    return STATUS_OK;
}

Status gpio_write(uint8_t pin, GpioValue value) {
    uint32_t gpio_pin = PAL_GPIO_PIN(pin);
    GPIO_TypeDef *base = PAL_GPIO_PORT(pin);
    GPIO_InitTypeDef conf = {
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pin = gpio_pin,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    uint32_t current_mode =
        (PAL_GPIO_PORT(pin)->MODER & PAL_GPIO_MODER_MASK(pin)) >>
        PAL_GPIO_MODER_POS(pin);
    if (current_mode != GPIO_MODE_OUTPUT_PP &&
        current_mode != GPIO_MODE_OUTPUT_OD) {
        HAL_GPIO_Init(base, &conf);
    }
    HAL_GPIO_WritePin(base, gpio_pin, value);
    return STATUS_OK;
}

GpioValue gpio_read(uint8_t pin) {
    uint32_t gpio_pin = PAL_GPIO_PIN(pin);
    GPIO_TypeDef *base = PAL_GPIO_PORT(pin);
    GPIO_InitTypeDef conf = {
        .Mode = GPIO_MODE_INPUT,
        .Pin = gpio_pin,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    if ((PAL_GPIO_PORT(pin)->MODER & PAL_GPIO_MODER_MASK(pin)) >>
            PAL_GPIO_MODER_POS(pin) !=
        GPIO_MODE_INPUT) {
        HAL_GPIO_Init(base, &conf);
    }
    uint32_t val = HAL_GPIO_ReadPin(base, gpio_pin);
    if (val == GPIO_HIGH) {
        return GPIO_HIGH;
    } else if (val == GPIO_LOW) {
        return GPIO_LOW;
    } else {
        return GPIO_ERR;
    }
}