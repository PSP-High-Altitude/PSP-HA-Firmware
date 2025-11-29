#include "i2c/i2c.h"

#include "pal_darkstar/board.h"
#include "stm32h7xx_hal.h"

#define I2C_PIN_AF_COUNT 10

enum {
    SDA = 1,
    SCL = 2,
};

typedef struct {
    uint8_t pin;
    uint8_t function;  // 0 - Invalid, 1 - SDA, 2 - SCL
    uint8_t af;
} I2cPinAf;

// List of all pin options for each peripheral
const I2cPinAf i2c_pin_af[5][I2C_PIN_AF_COUNT] = {
    {
        // I2C1
        {PIN_PB6, SCL, GPIO_AF4_I2C1},
        {PIN_PB7, SDA, GPIO_AF4_I2C1},
        {PIN_PB8, SCL, GPIO_AF4_I2C1},
        {PIN_PB9, SDA, GPIO_AF4_I2C1},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
    },
    {
        // I2C2
        {PIN_PB10, SCL, GPIO_AF4_I2C2},
        {PIN_PB11, SDA, GPIO_AF4_I2C2},
        {PIN_PF0, SDA, GPIO_AF4_I2C2},
        {PIN_PF1, SCL, GPIO_AF4_I2C2},
        {PIN_PH4, SCL, GPIO_AF4_I2C2},
        {PIN_PH5, SDA, GPIO_AF4_I2C2},
        {0},
        {0},
        {0},
        {0},
    },
    {
        // I2C3
        {PIN_PA8, SCL, GPIO_AF4_I2C3},
        {PIN_PC9, SDA, GPIO_AF4_I2C3},
        {PIN_PH7, SCL, GPIO_AF4_I2C3},
        {PIN_PH8, SDA, GPIO_AF4_I2C3},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
    },
    {
        // I2C4
        {PIN_PB6, SCL, GPIO_AF6_I2C4},
        {PIN_PB7, SDA, GPIO_AF6_I2C4},
        {PIN_PB8, SCL, GPIO_AF6_I2C4},
        {PIN_PB9, SDA, GPIO_AF6_I2C4},
        {PIN_PD12, SCL, GPIO_AF4_I2C4},
        {PIN_PD13, SDA, GPIO_AF4_I2C4},
        {PIN_PF14, SCL, GPIO_AF4_I2C4},
        {PIN_PF15, SDA, GPIO_AF4_I2C4},
        {PIN_PH11, SCL, GPIO_AF4_I2C4},
        {PIN_PH12, SDA, GPIO_AF4_I2C4},
    },
    {
        // I2C5
        {PIN_PA8, SCL, GPIO_AF6_I2C5},
        {PIN_PC9, SDA, GPIO_AF6_I2C5},
        {PIN_PC10, SDA, GPIO_AF4_I2C5},
        {PIN_PC11, SCL, GPIO_AF4_I2C5},
        {PIN_PF0, SDA, GPIO_AF6_I2C5},
        {PIN_PF1, SCL, GPIO_AF6_I2C5},
        {0},
        {0},
        {0},
        {0},
    },
};

static I2C_TypeDef *i2c_base[5] = {I2C1, I2C2, I2C3, I2C4, I2C5};
static I2C_HandleTypeDef i2c1_handle = {.State = 0};
static I2C_HandleTypeDef i2c2_handle = {.State = 0};
static I2C_HandleTypeDef i2c3_handle = {.State = 0};
static I2C_HandleTypeDef i2c4_handle = {.State = 0};
static I2C_HandleTypeDef i2c5_handle = {.State = 0};
static I2C_HandleTypeDef *i2c_handles[5] = {
    &i2c1_handle, &i2c2_handle, &i2c3_handle, &i2c4_handle, &i2c5_handle};

static uint32_t get_timings(I2cDevice *dev) {
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

static Status get_pin(uint8_t periph, uint8_t pin, uint8_t function,
                      uint32_t *af) {
    for (int i = 0; i < I2C_PIN_AF_COUNT; i++) {
        // Invalid function means we ran out of pins
        if (i2c_pin_af[periph][i].function == 0) {
            return STATUS_ERROR;
        }

        // Check if the pin is a match
        if (i2c_pin_af[periph][i].pin == pin &&
            i2c_pin_af[periph][i].function == function) {
            *af = i2c_pin_af[periph][i].af;
            return STATUS_OK;
        }
    }

    return STATUS_ERROR;
}

static Status i2c_setup(I2cDevice *dev) {
    // Check if the peripheral is valid
    if (dev->periph < P_I2C1 || dev->periph > P_I2C5) {
        return STATUS_PARAMETER_ERROR;
    }

    // Check if the peripheral is already initialized
    if (i2c_handles[dev->periph]->State != 0) {
        return STATUS_OK;
    }

    // Enable clock
    switch (dev->periph) {
        case P_I2C1:
            __HAL_RCC_I2C1_CLK_ENABLE();
            break;
        case P_I2C2:
            __HAL_RCC_I2C2_CLK_ENABLE();
            break;
        case P_I2C3:
            __HAL_RCC_I2C3_CLK_ENABLE();
            break;
        case P_I2C4:
            __HAL_RCC_I2C4_CLK_ENABLE();
            break;
        case P_I2C5:
            __HAL_RCC_I2C5_CLK_ENABLE();
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }

    // Get the base address of the peripheral
    I2C_TypeDef *base = i2c_base[dev->periph];

    // Create the pin configuration
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_OD,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_MEDIUM,
    };

    // Get and configure the SCL pin
    if (get_pin(dev->periph, dev->scl, SCL, &pin_conf.Alternate) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    pin_conf.Pin = PAL_GPIO_PIN(dev->scl);
    HAL_GPIO_Init(PAL_GPIO_PORT(dev->scl), &pin_conf);

    // Get and configure the SDA pin
    if (get_pin(dev->periph, dev->sda, SDA, &pin_conf.Alternate) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    pin_conf.Pin = PAL_GPIO_PIN(dev->sda);
    HAL_GPIO_Init(PAL_GPIO_PORT(dev->sda), &pin_conf);

    I2C_InitTypeDef init_conf = {
        .OwnAddress1 = 0,
        .DualAddressMode = I2C_DUALADDRESS_DISABLE,
        .OwnAddress2 = 0,
        .OwnAddress2Masks = I2C_OA2_NOMASK,
        .GeneralCallMode = I2C_GENERALCALL_DISABLE,
        .NoStretchMode = I2C_NOSTRETCH_DISABLE,
        .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
        .Timing = get_timings(dev),
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

Status i2c_write_verify(I2cDevice *device, uint8_t *tx_buf, size_t len) {
    uint8_t buf[8];

    // Max 7 byte write
    if (len > 7) {
        return STATUS_PARAMETER_ERROR;
    }

    // Initial write
    if (i2c_write(device, tx_buf, len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Read back
    if (i2c_write(device, tx_buf, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (i2c_read(device, buf, len) != STATUS_OK) {
        return STATUS_ERROR;
    }

    for (int i = 1; i < len; i++) {
        if (tx_buf[i] != buf[i - 1]) {
            return STATUS_ERROR;
        }
    }

    return STATUS_OK;
}

Status i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len) {
    if (i2c_setup(device) != STATUS_OK) {
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
    if (i2c_setup(device) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_I2C_Master_Receive(i2c_handles[device->periph],
                               device->address << 1, rx_buf, len,
                               100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}
