#ifndef SDMMMC_H
#define SDMMMC_H

// Standard library
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// PSPHAA library
#include "sd.h"
#include "status.h"

#define SdmmcPeriph SdPeriph
#define SdmmcSpeed SdSpeed
#define SD_SPEED_HIGH SD_SPEED_50MHz
#define SD_SPEED_DEFAULT SD_SPEED_25MHz
#define SdmmcDevice SdDevice

// From the HAL states
typedef enum {
    SD_CARD_READY,
    SD_CARD_IDENTIFICATION,
    SD_CARD_STANDBY,
    SD_CARD_TRANSFER,
    SD_CARD_SENDING,
    SD_CARD_RECEIVING,
    SD_CARD_PROGRAMMING,
    SD_CARD_DISCONNECTED,
    SD_CARD_ERROR,
} SdmmcState;

// From the HAL info
typedef struct {
    uint32_t CardType;
    uint32_t CardVersion;
    uint32_t Class;
    uint32_t RelCardAdd;
    uint32_t BlockNbr;
    uint32_t BlockSize;
    uint32_t LogBlockNbr;
    uint32_t LogBlockSize;
    uint32_t CardSpeed;
} SdmmcInfo;

Status sdmmc_setup(SdmmcDevice* dev);

Status sdmmc_init_card(SdmmcDevice* dev);

Status sdmmc_write_blocks(SdmmcDevice* dev, uint8_t* tx_buf,
                          uint32_t block_start, uint32_t num_blocks);

Status sdmmc_read_blocks(SdmmcDevice* dev, uint8_t* rx_buf,
                         uint32_t block_start, uint32_t num_blocks);

Status sdmmc_erase(SdmmcDevice* dev, uint32_t block_start, uint32_t block_end);

SdmmcState sdmmc_status(SdmmcDevice* dev);

Status sdmmc_info(SdmmcDevice* dev, SdmmcInfo* info);

#endif  // SDMMMC_H
