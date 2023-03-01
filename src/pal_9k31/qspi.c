#include "qspi/qspi.h"

#include "board.h"
#include "stm32g4xx_hal.h"

static QSPI_HandleTypeDef* qspi_handle = NULL;

static Status qspi_setup(QSpiDevice* dev) {
    if (qspi_handle != NULL) {
        return STATUS_OK;
    }
    QUADSPI_TypeDef* base = QUADSPI;
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    if (dev->bank == QSPI_BK1) {
        pin_conf.Alternate = GPIO_AF10_QUADSPI;
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE10];
        HAL_GPIO_Init(GPIOE, &pin_conf);  // CLK: pin PE10
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE11];
        HAL_GPIO_Init(GPIOE, &pin_conf);  // NCS: pin PE11
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE12];
        HAL_GPIO_Init(GPIOE, &pin_conf);  // IO0: pin PE12
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE13];
        HAL_GPIO_Init(GPIOE, &pin_conf);  // IO1: pin PE13
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE14];
        HAL_GPIO_Init(GPIOE, &pin_conf);  // IO2: pin PE14
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE15];
        HAL_GPIO_Init(GPIOE, &pin_conf);  // IO3: pin PE15
    } else {
        return STATUS_PARAMETER_ERROR;
    }
    uint32_t prescale = 0;
    switch (dev->clk) {
        case QSPI_SPEED_1MHz:
            prescale = 168;
            break;
        case QSPI_SPEED_2MHz:
            prescale = 84;
            break;
        case QSPI_SPEED_10MHz:
            prescale = 17;
            break;
        case QSPI_SPEED_20MHz:
            prescale = 9;
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }
    QSPI_InitTypeDef init_conf = {
        .ClockPrescaler = prescale,
        .FifoThreshold = 1,
        .SampleShifting = QSPI_SAMPLE_SHIFTING_NONE,
        .FlashSize = 30,
        .ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE,
        .ClockMode = QSPI_CLOCK_MODE_0,
        .FlashID = dev->bank ? QSPI_FLASH_ID_1 : QSPI_FLASH_ID_2,
        .DualFlash = QSPI_DUALFLASH_DISABLE,
    };
    QSPI_HandleTypeDef handle = {
        .Init = init_conf,
        .Instance = base,
    };
    HAL_QSPI_Init(&handle);
    qspi_handle = &handle;
    return STATUS_OK;
}

Status qspi_cmd(QSpiDevice* dev, QSPI_CommandTypeDef* cmd) {
    if (qspi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_QSPI_Command(qspi_handle, cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status qspi_write(QSpiDevice* dev, QSPI_CommandTypeDef* cmd, uint8_t* tx_buf) {
    if (qspi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_QSPI_Command(qspi_handle, cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_QSPI_Transmit(qspi_handle, tx_buf, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status qspi_read(QSpiDevice* dev, QSPI_CommandTypeDef* cmd, uint8_t* rx_buf) {
    if (qspi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_QSPI_Command(qspi_handle, cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_QSPI_Receive(qspi_handle, rx_buf, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}