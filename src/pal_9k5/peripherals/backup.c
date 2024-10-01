#include "backup.h"

Status backup_init() {
    // Enable backup RAM
    __HAL_RCC_BKPRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    // Enable backup regulator
    HAL_PWREx_EnableBkUpReg();
    while (!(PWR->CR2 & PWR_CR2_BRRDY_Msk));

    // Enable battery charging
    HAL_PWREx_EnableBatteryCharging(PWR_BATTERY_CHARGING_RESISTOR_5);

    return STATUS_OK;
}

Backup* get_backup_ptr() { return (Backup*)D3_BKPSRAM_BASE; }
