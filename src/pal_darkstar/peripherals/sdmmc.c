#include "sdmmc/sdmmc.h"

#include "board.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

MMC_HandleTypeDef hmmc1;
MMC_HandleTypeDef hmmc2;
MMC_HandleTypeDef* mmc_handles[] = {&hmmc1, &hmmc2};

Status sdmmc_setup(SdmmcDevice* dev) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    switch (dev->periph) {
        case P_SD1:
            __HAL_RCC_SDMMC1_CLK_ENABLE();
            __HAL_RCC_SDMMC1_FORCE_RESET();
            __HAL_RCC_SDMMC1_RELEASE_RESET();
            mmc_handles[0]->Instance = SDMMC1;
            NVIC_SetPriority(SDMMC1_IRQn, 0);
            NVIC_EnableIRQ(SDMMC1_IRQn);
            break;
        case P_SD2:
            __HAL_RCC_SDMMC2_CLK_ENABLE();
            __HAL_RCC_SDMMC2_FORCE_RESET();
            __HAL_RCC_SDMMC2_RELEASE_RESET();
            mmc_handles[1]->Instance = SDMMC2;
            GPIO_InitStruct.Alternate = GPIO_AF9_SDMMC2;
            GPIO_InitStruct.Pin = PAL_GPIO_PIN(PIN_PB14);  // D0
            HAL_GPIO_Init(PAL_GPIO_PORT(PIN_PB14), &GPIO_InitStruct);
            GPIO_InitStruct.Pin = PAL_GPIO_PIN(PIN_PB15);  // D1
            HAL_GPIO_Init(PAL_GPIO_PORT(PIN_PB15), &GPIO_InitStruct);
            GPIO_InitStruct.Pin = PAL_GPIO_PIN(PIN_PB3);  // D2
            HAL_GPIO_Init(PAL_GPIO_PORT(PIN_PB3), &GPIO_InitStruct);
            GPIO_InitStruct.Pin = PAL_GPIO_PIN(PIN_PB4);  // D3
            HAL_GPIO_Init(PAL_GPIO_PORT(PIN_PB4), &GPIO_InitStruct);
            GPIO_InitStruct.Pin = PAL_GPIO_PIN(PIN_PC1);  // CLK
            HAL_GPIO_Init(PAL_GPIO_PORT(PIN_PC1), &GPIO_InitStruct);
            GPIO_InitStruct.Pin = PAL_GPIO_PIN(PIN_PD7);  // CMD
            GPIO_InitStruct.Alternate = GPIO_AF11_SDMMC2;
            HAL_GPIO_Init(PAL_GPIO_PORT(PIN_PD7), &GPIO_InitStruct);
            NVIC_SetPriority(SDMMC2_IRQn, 0);
            NVIC_EnableIRQ(SDMMC2_IRQn);
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }

    mmc_handles[dev->periph]->Init.BusWide = SDMMC_BUS_WIDE_4B;
    mmc_handles[dev->periph]->Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    mmc_handles[dev->periph]->Init.ClockPowerSave =
        SDMMC_CLOCK_POWER_SAVE_DISABLE;
    mmc_handles[dev->periph]->Init.HardwareFlowControl =
        SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    mmc_handles[dev->periph]->Init.ClockDiv = 1;  // 80Mhz -> 40Mhz

    if (HAL_MMC_Init(mmc_handles[dev->periph]) != HAL_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

Status sdmmc_init_card(SdmmcDevice* dev) {
    if (HAL_MMC_InitCard(mmc_handles[dev->periph]) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

Status sdmmc_write_blocks(SdmmcDevice* dev, uint8_t* tx_buf,
                          uint32_t block_start, uint32_t num_blocks) {
    if (HAL_MMC_WriteBlocks_DMA(mmc_handles[dev->periph], tx_buf, block_start,
                                num_blocks) != HAL_OK) {
        return STATUS_ERROR;
    }
    uint64_t start = MILLIS();
    while (mmc_handles[dev->periph]->State != HAL_MMC_STATE_READY ||
           HAL_MMC_GetCardState(mmc_handles[dev->periph]) !=
               HAL_SD_CARD_TRANSFER) {
        if (MILLIS() - start > 500) {
            return STATUS_ERROR;
        }
        DELAY(0);
    }
    return STATUS_OK;
}

Status sdmmc_read_blocks(SdmmcDevice* dev, uint8_t* rx_buf,
                         uint32_t block_start, uint32_t num_blocks) {
    if (HAL_MMC_ReadBlocks_DMA(mmc_handles[dev->periph], rx_buf, block_start,
                               num_blocks) != HAL_OK) {
        return STATUS_ERROR;
    }
    uint64_t start = MILLIS();
    while (mmc_handles[dev->periph]->State != HAL_MMC_STATE_READY ||
           HAL_MMC_GetCardState(mmc_handles[dev->periph]) !=
               HAL_SD_CARD_TRANSFER) {
        if (MILLIS() - start > 500) {
            return STATUS_ERROR;
        }
        DELAY(0);
    }
    return STATUS_OK;
}

Status sdmmc_erase(SdmmcDevice* dev, uint32_t block_start, uint32_t block_end) {
    if (HAL_MMC_Erase(mmc_handles[dev->periph], block_start, block_end) !=
        HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

SdmmcState sdmmc_status(SdmmcDevice* dev) {
    switch (HAL_MMC_GetCardState(mmc_handles[dev->periph])) {
        case HAL_MMC_CARD_READY:
            return SD_CARD_READY;
        case HAL_MMC_CARD_IDENTIFICATION:
            return SD_CARD_IDENTIFICATION;
        case HAL_MMC_CARD_STANDBY:
            return SD_CARD_STANDBY;
        case HAL_MMC_CARD_TRANSFER:
            return SD_CARD_TRANSFER;
        case HAL_MMC_CARD_SENDING:
            return SD_CARD_SENDING;
        case HAL_MMC_CARD_RECEIVING:
            return SD_CARD_RECEIVING;
        case HAL_MMC_CARD_PROGRAMMING:
            return SD_CARD_PROGRAMMING;
        case HAL_MMC_CARD_DISCONNECTED:
            return SD_CARD_DISCONNECTED;
        case HAL_MMC_CARD_ERROR:
            return SD_CARD_ERROR;
        default:
            return SD_CARD_ERROR;
    }
}

Status sdmmc_info(SdmmcDevice* dev, SdmmcInfo* info) {
    info->CardType = mmc_handles[dev->periph]->MmcCard.CardType;
    info->CardVersion = 0;
    info->Class = mmc_handles[dev->periph]->MmcCard.Class;
    info->RelCardAdd = mmc_handles[dev->periph]->MmcCard.RelCardAdd;
    info->BlockNbr = mmc_handles[dev->periph]->MmcCard.BlockNbr;
    info->BlockSize = mmc_handles[dev->periph]->MmcCard.BlockSize;
    info->LogBlockNbr = mmc_handles[dev->periph]->MmcCard.LogBlockNbr;
    info->LogBlockSize = mmc_handles[dev->periph]->MmcCard.LogBlockSize;
    info->CardSpeed = 0;
    return STATUS_OK;
}

void SDMMC1_IRQHandler() { HAL_MMC_IRQHandler(&hmmc1); }

void SDMMC2_IRQHandler() { HAL_MMC_IRQHandler(&hmmc2); }