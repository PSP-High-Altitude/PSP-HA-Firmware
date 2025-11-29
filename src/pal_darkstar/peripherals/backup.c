#include "backup/backup.h"

#include "stm32h7xx_hal.h"

#define BKPSRAM_SIZE 4096  // bytes

_Static_assert(sizeof(Backup) < BKPSRAM_SIZE, "Backup does not fit in BKPSRAM");

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

Backup* backup_get_ptr() { return (Backup*)D3_BKPSRAM_BASE; }

Status backup_invalidate() {
    memset(backup_get_ptr(), 0x00, sizeof(Backup));
    return STATUS_OK;
}
