#include "backup.h"

void init_backup() {
    // Enable backup RAM
    __HAL_RCC_BKPRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    // Enable backup regulator
    HAL_PWREx_EnableBkUpReg();
    while (!(PWR->CR2 & PWR_CR2_BRRDY_Msk));
}

Backup* get_backup_ptr() { return (Backup*)D3_BKPSRAM_BASE; }
