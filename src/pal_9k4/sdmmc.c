#include "sdmmc/sdmmc.h"

#include "FreeRTOS.h"
#include "board.h"
#include "gpio/gpio.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

static SD_HandleTypeDef sdmmc1_handle;
static SD_HandleTypeDef sdmmc2_handle;
static SD_HandleTypeDef* sdmmc_handles[] = {&sdmmc1_handle, &sdmmc2_handle};

Status sdmmc_setup(SdmmcDevice* dev) {
    if (dev->periph < 0 || dev->periph > 1) {
        return STATUS_PARAMETER_ERROR;
    }
    if (dev->clk != SD_SPEED_DEFAULT && dev->clk != SD_SPEED_HIGH &&
        dev->clk != SD_SPEED_12_5MHz) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_SD_GetState(sdmmc_handles[dev->periph]) != HAL_SD_STATE_RESET) {
        return STATUS_OK;
    }
    SD_TypeDef* base = NULL;
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    switch (dev->periph) {
        case P_SD1:
            HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
            base = SDMMC1;
            // Disable SPI first
            gpio_mode(PIN_PE4, GPIO_INPUT_PULLUP);
            gpio_mode(PIN_PE2, GPIO_INPUT_PULLUP);
            gpio_mode(PIN_PE5, GPIO_INPUT_PULLUP);
            gpio_mode(PIN_PE6, GPIO_INPUT_PULLUP);

            pin_conf.Alternate = GPIO_AF12_SDMMC1;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PC9] |
                           GPIO_PIN_TO_NUM[PIN_PC10] |
                           GPIO_PIN_TO_NUM[PIN_PC11] |
                           GPIO_PIN_TO_NUM[PIN_PC12];  // D1: PC9, D2: PC10,
                                                       // D3: PC11, CLK: PC12
            HAL_GPIO_DeInit(GPIOC, pin_conf.Pin);
            HAL_GPIO_Init(GPIOC, &pin_conf);

            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB13];  // D0: PB13
            HAL_GPIO_DeInit(GPIOB, pin_conf.Pin);
            HAL_GPIO_Init(GPIOB, &pin_conf);

            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PD2];  // CMD: PD2
            HAL_GPIO_DeInit(GPIOD, pin_conf.Pin);
            HAL_GPIO_Init(GPIOD, &pin_conf);
            break;
        case P_SD2:
            return STATUS_PARAMETER_ERROR;
            break;
        default:
            return STATUS_PARAMETER_ERROR;
            break;
    }
    uint32_t prescale = 0;
    switch (dev->clk) {
        case SD_SPEED_HIGH:
            prescale = 0;
            break;
        case SD_SPEED_DEFAULT:
            prescale = 1;
            break;
        case SD_SPEED_12_5MHz:
            prescale = 2;
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }
    SD_InitTypeDef init_conf = {
        .ClockEdge = SDMMC_CLOCK_EDGE_RISING,
        .ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE,
        .BusWide = SDMMC_BUS_WIDE_4B,
        .HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE,
        .ClockDiv = prescale,
    };
    SD_HandleTypeDef* handle = sdmmc_handles[dev->periph];
    handle->Init = init_conf;
    handle->Instance = base;
    if (HAL_SD_Init(handle) != HAL_OK) {
        return STATUS_ERROR;
    }
    if (HAL_SD_GetCardState(sdmmc_handles[dev->periph]) !=
        HAL_SD_CARD_TRANSFER) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status sdmmc_init_card(SdmmcDevice* dev) {
    if (HAL_SD_InitCard(sdmmc_handles[dev->periph]) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status sdmmc_write_blocks(SdmmcDevice* dev, uint8_t* tx_buf,
                          uint32_t block_start, uint32_t num_blocks) {
    if (HAL_SD_WriteBlocks_DMA(sdmmc_handles[dev->periph], tx_buf, block_start,
                               num_blocks) != HAL_OK) {
        return STATUS_ERROR;
    }
    uint64_t start = MILLIS();
    while (sdmmc_handles[dev->periph]->State != HAL_SD_STATE_READY ||
           HAL_SD_GetCardState(sdmmc_handles[dev->periph]) !=
               HAL_SD_CARD_TRANSFER) {
        if (MILLIS() - start > 500) {
            return STATUS_ERROR;
        }
        DELAY(1);
    }
    return STATUS_OK;
}

Status sdmmc_read_blocks(SdmmcDevice* dev, uint8_t* rx_buf,
                         uint32_t block_start, uint32_t num_blocks) {
    if (HAL_SD_ReadBlocks_DMA(sdmmc_handles[dev->periph], rx_buf, block_start,
                              num_blocks) != HAL_OK) {
        return STATUS_ERROR;
    }
    uint64_t start = MILLIS();
    while (sdmmc_handles[dev->periph]->State != HAL_SD_STATE_READY ||
           HAL_SD_GetCardState(sdmmc_handles[dev->periph]) !=
               HAL_SD_CARD_TRANSFER) {
        if (MILLIS() - start > 500) {
            return STATUS_ERROR;
        }
        DELAY(1);
    }
    return STATUS_OK;
}

Status sdmmc_erase(SdmmcDevice* dev, uint32_t block_start, uint32_t block_end) {
    if (HAL_SD_Erase(sdmmc_handles[dev->periph], block_start, block_end) !=
        HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

SdmmcState sdmmc_status(SdmmcDevice* dev) {
    HAL_SD_CardStateTypeDef state =
        HAL_SD_GetCardState(sdmmc_handles[dev->periph]);
    switch (state) {
        case HAL_SD_CARD_READY:
            return SD_CARD_READY;
            break;
        case HAL_SD_CARD_IDENTIFICATION:
            return SD_CARD_IDENTIFICATION;
            break;
        case HAL_SD_CARD_STANDBY:
            return SD_CARD_STANDBY;
            break;
        case HAL_SD_CARD_TRANSFER:
            return SD_CARD_TRANSFER;
            break;
        case HAL_SD_CARD_SENDING:
            return SD_CARD_SENDING;
            break;
        case HAL_SD_CARD_RECEIVING:
            return SD_CARD_RECEIVING;
            break;
        case HAL_SD_CARD_PROGRAMMING:
            return SD_CARD_PROGRAMMING;
            break;
        case HAL_SD_CARD_DISCONNECTED:
            return SD_CARD_DISCONNECTED;
            break;
        case HAL_SD_CARD_ERROR:
            return SD_CARD_ERROR;
            break;
    }
    return SD_CARD_ERROR;
}

Status sdmmc_info(SdmmcDevice* dev, SdmmcInfo* info) {
    HAL_SD_CardInfoTypeDef hal_info;
    if (HAL_SD_GetCardInfo(sdmmc_handles[dev->periph], &hal_info) != HAL_OK) {
        return STATUS_ERROR;
    }
    SdmmcInfo ret = {
        .CardType = hal_info.CardType,
        .CardVersion = hal_info.CardVersion,
        .Class = hal_info.Class,
        .RelCardAdd = hal_info.RelCardAdd,
        .BlockNbr = hal_info.BlockNbr,
        .BlockSize = hal_info.BlockSize,
        .LogBlockNbr = hal_info.LogBlockNbr,
        .LogBlockSize = hal_info.LogBlockSize,
        .CardSpeed = hal_info.CardSpeed,
    };
    *info = ret;
    return STATUS_OK;
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef* hsd) {
    hsd->State = HAL_SD_STATE_READY;
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef* hsd) {
    hsd->State = HAL_SD_STATE_READY;
}

void SDMMC1_IRQHandler(void) { HAL_SD_IRQHandler(&sdmmc1_handle); }