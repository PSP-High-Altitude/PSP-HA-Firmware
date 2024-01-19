#include "ospi.h"

#include "board.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

static OSPI_HandleTypeDef ospi1_handle = {.State = 0};
static OSPI_HandleTypeDef* ospi_handles[] = {&ospi1_handle};

static Status ospi_setup(OSpiDevice* dev) {
    if (ospi_handles[0]->State != 0) {
        return STATUS_OK;
    }
    OCTOSPI_TypeDef* base = OCTOSPI1;
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    if (dev->bank == OSPI_PORT1_7_4) {
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
        case OSPI_SPEED_1MHz:
            prescale = 80;
            break;
        case OSPI_SPEED_5MHz:
            prescale = 16;
            break;
        case OSPI_SPEED_10MHz:
            prescale = 8;
            break;
        case OSPI_SPEED_20MHz:
            prescale = 4;
            break;
        case OSPI_SPEED_40MHz:
            prescale = 2;
            break;
        case OSPI_SPEED_80MHz:
            prescale = 1;
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }
    OSPI_InitTypeDef init_conf = {
        .ClockPrescaler = prescale,
        .FifoThreshold = 1,
        .DualQuad = HAL_OSPI_DUALQUAD_DISABLE,
        .SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE,
        .DeviceSize = 30,
        .MemoryType = HAL_OSPI_MEMTYPE_MICRON,
        .ChipSelectHighTime = 1,
        .ClockMode = HAL_OSPI_CLOCK_MODE_0,
        .WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED,
        .DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE,
        .ChipSelectBoundary = 0,
        .DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED,
        .FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE,
        .MaxTran = 0,
        .Refresh = 0,
    };
    OSPI_HandleTypeDef* handle = ospi_handles[0];
    (*handle).Init = init_conf;
    (*handle).Instance = base;
    if (HAL_OSPI_Init(handle) != HAL_OK) {
        return STATUS_ERROR;
    }
    OSPIM_CfgTypeDef sOspiManagerCfg = {0};
    sOspiManagerCfg.ClkPort = 1;
    sOspiManagerCfg.NCSPort = 1;
    sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_HIGH;
    if (HAL_OSPIM_Config(ospi_handles[0], &sOspiManagerCfg,
                         HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status ospi_cmd(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd) {
    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_OSPI_Command(ospi_handles[0], cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status ospi_auto_poll_cmd(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd,
                          OSPI_AutoPollingTypeDef* cfg) {
    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    HAL_StatusTypeDef status;
    if (HAL_OSPI_Command(ospi_handles[0], cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if ((status = HAL_OSPI_AutoPolling(ospi_handles[0], cfg, 100)) != HAL_OK) {
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
        if (HAL_OSPI_Command(ospi_handles[0], cmd, 100) != HAL_OK) {
            return STATUS_ERROR;
        }
        if (HAL_OSPI_Receive(ospi_handles[0], rx_buf, 100) != HAL_OK) {
            return STATUS_ERROR;
        }
        rx_data = (((uint32_t)rx_buf[0] << 24) | ((uint32_t)rx_buf[1] << 16) |
                   ((uint32_t)rx_buf[2] << 8) | rx_buf[3]);
        printf("status: %lx\n", rx_data);
    } while ((rx_data & cfg->Mask) != cfg->Match);
    return STATUS_OK;
    */
}

Status ospi_write(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd,
                  uint8_t* tx_buf) {
    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_OSPI_Command(ospi_handles[0], cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_OSPI_Transmit(ospi_handles[0], tx_buf, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status ospi_read(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd,
                 uint8_t* rx_buf) {
    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_OSPI_Command(ospi_handles[0], cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_OSPI_Receive(ospi_handles[0], rx_buf, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}