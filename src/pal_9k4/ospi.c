#include "ospi.h"

#include "board.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

static OSPI_HandleTypeDef qspi1_handle = {.State = 0};
static OSPI_HandleTypeDef* qspi_handles[] = {&qspi1_handle};

static Status qspi_setup(QSpiDevice* dev) {
    if (qspi_handles[0]->State != 0) {
        return STATUS_OK;
    }
    OCTOSPI_TypeDef* base = OCTOSPI1;
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    if (dev->bank == QSPI_PORT1_7_4) {
        pin_conf.Alternate = GPIO_AF9_OCTOSPIM_P1;
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PF10];
        HAL_GPIO_Init(GPIOF, &pin_conf);  // CLK: pin PF10
        pin_conf.Alternate = GPIO_AF10_OCTOSPIM_P1;
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PC1];
        HAL_GPIO_Init(GPIOC, &pin_conf);  // IO0: pin PC1
        pin_conf.Alternate = GPIO_AF4_OCTOSPIM_P1;
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PC2] | GPIO_PIN_TO_NUM[PIN_PC3];
        HAL_GPIO_Init(GPIOC, &pin_conf);  // IO1: pin PC2, IO2: pin PC3
        pin_conf.Alternate = GPIO_AF10_OCTOSPIM_P1;
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE10];
        HAL_GPIO_Init(GPIOE, &pin_conf);  // IO3: pin PE10
        pin_conf.Alternate = GPIO_AF11_OCTOSPIM_P1;
        pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE11];
        HAL_GPIO_Init(GPIOE, &pin_conf);  // NCS: pin PE11
    } else {
        return STATUS_PARAMETER_ERROR;
    }
    uint32_t prescale = 0;
    switch (dev->clk) {
        case QSPI_SPEED_1MHz:
            prescale = 80;
            break;
        case QSPI_SPEED_5MHz:
            prescale = 16;
            break;
        case QSPI_SPEED_10MHz:
            prescale = 8;
            break;
        case QSPI_SPEED_20MHz:
            prescale = 4;
            break;
        case QSPI_SPEED_40MHz:
            prescale = 2;
            break;
        case QSPI_SPEED_80MHz:
            prescale = 1;
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }
    QSPI_InitTypeDef init_conf = {
        .ClockPrescaler = prescale,
        .FifoThreshold = 1,
        .SampleShifting = QSPI_SAMPLE_SHIFTING_NONE,
        .FlashSize = 30,
        .ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE,
        .ClockMode = QSPI_CLOCK_MODE_0,
        .FlashID = dev->bank ? QSPI_FLASH_ID_2 : QSPI_FLASH_ID_1,
        .DualFlash = QSPI_DUALFLASH_DISABLE,
    };
    QSPI_HandleTypeDef* handle = qspi_handles[0];
    (*handle).Init = init_conf;
    (*handle).Instance = base;
    if (HAL_QSPI_Init(handle) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status qspi_cmd(QSpiDevice* dev, QSPI_CommandTypeDef* cmd) {
    if (qspi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_QSPI_Command(qspi_handles[0], cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status qspi_auto_poll_cmd(QSpiDevice* dev, QSPI_CommandTypeDef* cmd,
                          QSPI_AutoPollingTypeDef* cfg) {
    if (qspi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    HAL_StatusTypeDef status;
    if ((status = HAL_QSPI_AutoPolling(qspi_handles[0], cmd, cfg, 100)) !=
        HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;

    /*
    uint64_t start_time = MILLIS();
    uint8_t rx_buf[4] = {0, 0, 0, 0};
    uint32_t rx_data;
    do {
        if (MILLIS() - start_time > 100) {
            return STATUS_TIMEOUT;
        }
        if (HAL_QSPI_Command(qspi_handles[0], cmd, 100) != HAL_OK) {
            return STATUS_ERROR;
        }
        if (HAL_QSPI_Receive(qspi_handles[0], rx_buf, 100) != HAL_OK) {
            return STATUS_ERROR;
        }
        rx_data = (((uint32_t)rx_buf[0] << 24) | ((uint32_t)rx_buf[1] << 16) |
                   ((uint32_t)rx_buf[2] << 8) | rx_buf[3]);
        printf("status: %lx\n", rx_data);
    } while ((rx_data & cfg->Mask) != cfg->Match);
    return STATUS_OK;
    */
}

Status qspi_write(QSpiDevice* dev, QSPI_CommandTypeDef* cmd, uint8_t* tx_buf) {
    if (qspi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_QSPI_Command(qspi_handles[0], cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_QSPI_Transmit(qspi_handles[0], tx_buf, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status qspi_read(QSpiDevice* dev, QSPI_CommandTypeDef* cmd, uint8_t* rx_buf) {
    if (qspi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_QSPI_Command(qspi_handles[0], cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_QSPI_Receive(qspi_handles[0], rx_buf, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}