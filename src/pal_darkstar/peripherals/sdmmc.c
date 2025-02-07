#include "sdmmc/sdmmc.h"

Status sdmmc_setup(SdmmcDevice* dev) { return STATUS_OK; }

Status sdmmc_init_card(SdmmcDevice* dev) { return STATUS_OK; }

Status sdmmc_write_blocks(SdmmcDevice* dev, uint8_t* tx_buf,
                          uint32_t block_start, uint32_t num_blocks) {
    return STATUS_OK;
}

Status sdmmc_read_blocks(SdmmcDevice* dev, uint8_t* rx_buf,
                         uint32_t block_start, uint32_t num_blocks) {
    return STATUS_OK;
}

Status sdmmc_erase(SdmmcDevice* dev, uint32_t block_start, uint32_t block_end) {
    return STATUS_OK;
}

SdmmcState sdmmc_status(SdmmcDevice* dev) { return SD_CARD_DISCONNECTED; }

Status sdmmc_info(SdmmcDevice* dev, SdmmcInfo* info) { return STATUS_OK; }