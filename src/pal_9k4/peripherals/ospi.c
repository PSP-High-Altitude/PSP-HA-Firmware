#include "ospi.h"

#include "pal_9k4/board.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

static OSPI_HandleTypeDef ospi1_handle = {.State = 0};
static MDMA_HandleTypeDef hmdma_octospi1_fifo_th;

static Status ospi_setup(OSpiDevice* dev) {
    if (ospi1_handle.State != 0) {
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
        .DeviceSize = 27,
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
    ospi1_handle.Init = init_conf;
    ospi1_handle.Instance = base;
    if (HAL_OSPI_Init(&ospi1_handle) != HAL_OK) {
        return STATUS_ERROR;
    }
    OSPIM_CfgTypeDef sOspiManagerCfg = {0};
    sOspiManagerCfg.ClkPort = 1;
    sOspiManagerCfg.NCSPort = 1;
    sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_HIGH;
    sOspiManagerCfg.DQSPort = 0;
    if (HAL_OSPIM_Config(&ospi1_handle, &sOspiManagerCfg,
                         HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return STATUS_ERROR;
    }

    hmdma_octospi1_fifo_th.Instance = MDMA_Channel0;
    hmdma_octospi1_fifo_th.Init.Request = MDMA_REQUEST_OCTOSPI1_FIFO_TH;
    hmdma_octospi1_fifo_th.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
    hmdma_octospi1_fifo_th.Init.Priority = MDMA_PRIORITY_LOW;
    hmdma_octospi1_fifo_th.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    hmdma_octospi1_fifo_th.Init.SourceInc = MDMA_SRC_INC_BYTE;
    hmdma_octospi1_fifo_th.Init.DestinationInc = MDMA_DEST_INC_BYTE;
    hmdma_octospi1_fifo_th.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
    hmdma_octospi1_fifo_th.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
    hmdma_octospi1_fifo_th.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    hmdma_octospi1_fifo_th.Init.BufferTransferLength = 1;
    hmdma_octospi1_fifo_th.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    hmdma_octospi1_fifo_th.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
    hmdma_octospi1_fifo_th.Init.SourceBlockAddressOffset = 0;
    hmdma_octospi1_fifo_th.Init.DestBlockAddressOffset = 0;
    if (HAL_MDMA_Init(&hmdma_octospi1_fifo_th) != HAL_OK) {
        return STATUS_ERROR;
    }

    if (HAL_MDMA_ConfigPostRequestMask(&hmdma_octospi1_fifo_th, 0, 0) !=
        HAL_OK) {
        return STATUS_ERROR;
    }

    __HAL_LINKDMA(&ospi1_handle, hmdma, hmdma_octospi1_fifo_th);

    HAL_NVIC_SetPriority(OCTOSPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OCTOSPI1_IRQn);
    HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(MDMA_IRQn);

    return STATUS_OK;
}

Status ospi_cmd(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd) {
    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_OSPI_Command(&ospi1_handle, cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status ospi_auto_poll_cmd(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd,
                          OSPI_AutoPollingTypeDef* cfg) {
    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_OSPI_Command(&ospi1_handle, cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_OSPI_AutoPolling(&ospi1_handle, cfg, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status ospi_write(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd,
                  uint8_t* tx_buf) {
    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_OSPI_Command(&ospi1_handle, cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_OSPI_Transmit_DMA(&ospi1_handle, tx_buf) != HAL_OK) {
        return STATUS_ERROR;
    }
    uint64_t start_time = MILLIS();
    while (HAL_OSPI_GetState(&ospi1_handle) != HAL_OSPI_STATE_READY) {
        if (MILLIS() - start_time > 500) {
            return STATUS_ERROR;
        }
        DELAY_MICROS(100);
    }
    return STATUS_OK;
}

Status ospi_read(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd,
                 uint8_t* rx_buf) {
    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_OSPI_Command(&ospi1_handle, cmd, 100) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_OSPI_Receive_DMA(&ospi1_handle, rx_buf) != HAL_OK) {
        return STATUS_ERROR;
    }
    uint64_t start_time = MILLIS();
    while (HAL_OSPI_GetState(&ospi1_handle) != HAL_OSPI_STATE_READY) {
        if (MILLIS() - start_time > 500) {
            return STATUS_ERROR;
        }
        DELAY_MICROS(100);
    }
    return STATUS_OK;
}

void OCTOSPI1_IRQHandler(void) { HAL_OSPI_IRQHandler(&ospi1_handle); }

void MDMA_IRQHandler(void) { HAL_MDMA_IRQHandler(&hmdma_octospi1_fifo_th); }