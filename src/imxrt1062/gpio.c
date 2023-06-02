#include "gpio/gpio.h"

#include "imxrt1062/MIMXRT1062/drivers/fsl_gpio.h"
#include "imxrt1062/MIMXRT1062/drivers/fsl_iomuxc.h"
#include "teensy_41/board.h"

Status gpio_mode(uint8_t pin, GpioMode mode) {
    IOMUXC_SetPinMux(IOMUXC_PIN_TO_MUX[pin], 5, 0, 0, 0, 0);
    uint8_t gpio_pin = GPIO_PIN_TO_NUM[pin];
    GPIO_Type *base = GPIO_PIN_TO_BASE[pin];
    gpio_pin_config_t conf = {
        .direction = mode,
        .interruptMode = kGPIO_NoIntmode,
        .outputLogic = 0,
    };
    GPIO_PinInit(base, gpio_pin, &conf);
    return STATUS_OK;
}

Status gpio_write(uint8_t pin, GpioValue value) {
    IOMUXC_SetPinMux(IOMUXC_PIN_TO_MUX[pin], 5, 0, 0, 0, 0);
    uint8_t gpio_pin = GPIO_PIN_TO_NUM[pin];
    GPIO_Type *base = GPIO_PIN_TO_BASE[pin];
    gpio_pin_config_t conf = {
        .direction = GPIO_OUTPUT,
        .interruptMode = kGPIO_NoIntmode,
        .outputLogic = value,
    };
    GPIO_PinInit(base, gpio_pin, &conf);
    return STATUS_OK;
}

GpioValue gpio_read(uint8_t pin) {
    IOMUXC_SetPinMux(IOMUXC_PIN_TO_MUX[pin], 5, 0, 0, 0, 0);
    uint8_t gpio_pin = GPIO_PIN_TO_NUM[pin];
    GPIO_Type *base = GPIO_PIN_TO_BASE[pin];
    gpio_pin_config_t conf = {
        .direction = GPIO_INPUT,
        .interruptMode = kGPIO_NoIntmode,
        .outputLogic = 0,
    };
    GPIO_PinInit(base, pin, &conf);
    uint32_t val = GPIO_PinRead(base, gpio_pin);
    if (val == GPIO_HIGH) {
        return GPIO_HIGH;
    } else if (val == GPIO_LOW) {
        return GPIO_LOW;
    } else {
        return GPIO_ERR;
    }
}