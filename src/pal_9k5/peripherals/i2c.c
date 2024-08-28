#include "i2c/i2c.h"

#include "pal_9k4/board.h"
#include "stm32h7xx_hal.h"

static I2C_HandleTypeDef i2c1_handle = {.State = 0};
static I2C_HandleTypeDef i2c2_handle = {.State = 0};
static I2C_HandleTypeDef i2c3_handle = {.State = 0};
static I2C_HandleTypeDef i2c4_handle = {.State = 0};
static I2C_HandleTypeDef i2c5_handle = {.State = 0};
static I2C_HandleTypeDef i2c6_handle = {.State = 0};
static I2C_HandleTypeDef *i2c_handles[6] = {&i2c1_handle, &i2c2_handle,
                                            &i2c3_handle, &i2c4_handle,
                                            &i2c5_handle, &i2c6_handle};

static uint32_t getTimings(I2cDevice *dev) {
    switch (dev->clk) {
        case I2C_SPEED_STANDARD:
            return 0x10707DBC;
        case I2C_SPEED_FAST:
            return 0x00602173;
        case I2C_SPEED_FAST_PLUS:
            return 0x00300B29;
        default:
            return 0;
    }
}

static Status i2cSetup(I2cDevice *dev) {
    if (dev->periph < 0 || dev->periph > 3) {
        return STATUS_PARAMETER_ERROR;
    }
    if (i2c_handles[dev->periph]->State != 0) {
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
            pin_conf.Alternate = GPIO_AF4_I2C1;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB9];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // SDA: pin PB9
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB8];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // SCL: pin PB8
            break;
        case P_I2C2:
            base = I2C2;
            pin_conf.Alternate = GPIO_AF4_I2C2;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB11];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // SDA: pin PB11
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB10];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // SCL: pin PB10
            break;
        case P_I2C3:
            return STATUS_PARAMETER_ERROR;
            break;
        case P_I2C4:
            base = I2C4;
            pin_conf.Alternate = GPIO_AF6_I2C4;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB7];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // SDA: pin PB7
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB6];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // SCL: pin PB6
            break;
        case P_I2C5:
            return STATUS_PARAMETER_ERROR;
            break;
        case P_I2C6:
            return STATUS_PARAMETER_ERROR;
            break;
    }
    I2C_InitTypeDef init_conf = {
        .OwnAddress1 = 0,
        .DualAddressMode = I2C_DUALADDRESS_DISABLE,
        .OwnAddress2 = 0,
        .OwnAddress2Masks = I2C_OA2_NOMASK,
        .GeneralCallMode = I2C_GENERALCALL_DISABLE,
        .NoStretchMode = I2C_NOSTRETCH_DISABLE,
        .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
        .Timing = getTimings(dev),
    };
    if (!init_conf.Timing) {
        return STATUS_PARAMETER_ERROR;
    }
    I2C_HandleTypeDef *handle = i2c_handles[dev->periph];
    (*handle).Init = init_conf;
    (*handle).Instance = base;
    if (HAL_I2C_Init(handle) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_I2CEx_ConfigAnalogFilter(handle, I2C_ANALOGFILTER_ENABLE) !=
        HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_I2CEx_ConfigDigitalFilter(handle, 0) != HAL_OK) {
        return STATUS_ERROR;
    }
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