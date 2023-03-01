#include "i2c/i2c.h"

#include "pal_9k31/board.h"
#include "stm32g4xx_hal.h"

static I2C_HandleTypeDef *i2c_handles[] = {NULL, NULL, NULL, NULL};

static uint32_t getTimings(I2cDevice *dev) {
    uint32_t presc;
    uint32_t scll;
    uint32_t sclh;
    uint32_t sdadel;
    uint32_t scldel;
    switch (dev->clk) {
        case I2C_SPEED_STANDARD:
            presc = 3;
            scll = 0x13;
            sclh = 0xF;
            sdadel = 0x2;
            scldel = 0x4;
            return (presc << 28) | (scldel << 20) | (sdadel << 16) |
                   (sclh << 8) | scll;
            break;
        case I2C_SPEED_FAST:
            presc = 1;
            scll = 0x9;
            sclh = 0x3;
            sdadel = 0x2;
            scldel = 0x3;
            return (presc << 28) | (scldel << 20) | (sdadel << 16) |
                   (sclh << 8) | scll;
            break;
        case I2C_SPEED_FAST_PLUS:
            presc = 0;
            scll = 0x4;
            sclh = 0x2;
            sdadel = 0x0;
            scldel = 0x2;
            return (presc << 28) | (scldel << 20) | (sdadel << 16) |
                   (sclh << 8) | scll;
            break;
        default:
            return 0;
    }
}

static Status i2cSetup(I2cDevice *dev) {
    if (dev->periph < 0 || dev->periph > 3) {
        return STATUS_PARAMETER_ERROR;
    }
    if (i2c_handles[dev->periph] != NULL) {
        return STATUS_OK;
    }
    I2C_TypeDef *base = NULL;
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_OD,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_MEDIUM,
    };
    switch (dev->periph) {
        case P_I2C1:
            base = I2C1;
            return STATUS_PARAMETER_ERROR;
            break;
        case P_I2C2:
            base = I2C2;
            pin_conf.Alternate = GPIO_AF4_I2C2;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PA8];
            HAL_GPIO_Init(GPIOA, &pin_conf);  // SDA: pin PA8
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PA9];
            HAL_GPIO_Init(GPIOA, &pin_conf);  // SCL: pin PA9
            break;
        case P_I2C3:
            base = I2C3;
            pin_conf.Alternate = GPIO_AF8_I2C3;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PC8];
            HAL_GPIO_Init(GPIOC, &pin_conf);  // SCL: pin PC8
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PC9];
            HAL_GPIO_Init(GPIOC, &pin_conf);  // SDA: pin PC9
            break;
        case P_I2C4:
            base = I2C4;
            pin_conf.Alternate = GPIO_AF8_I2C4;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PC6];
            HAL_GPIO_Init(GPIOC, &pin_conf);  // SCL: pin PC6
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PC7];
            HAL_GPIO_Init(GPIOC, &pin_conf);  // SDA: pin PC7
            break;
    }
    I2C_InitTypeDef init_conf = {
        .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
        .Timing = getTimings(dev),
    };
    if (!init_conf.Timing) {
        return STATUS_PARAMETER_ERROR;
    }
    I2C_HandleTypeDef handle = {
        .Init = init_conf,
        .Instance = base,
        .Mode = HAL_I2C_MODE_MASTER,
    };
    HAL_I2C_Init(&handle);
    i2c_handles[dev->periph] = &handle;
    return STATUS_OK;
}

Status i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len) {
    if (i2cSetup(device) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_I2C_Master_Transmit(i2c_handles[device->periph],
                                device->address << 1, tx_buf, len,
                                100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len) {
    if (i2cSetup(device) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_I2C_Master_Receive(i2c_handles[device->periph],
                               device->address << 1, rx_buf, len,
                               100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}