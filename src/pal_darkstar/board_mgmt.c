#include "board_mgmt.h"

#include "backup/backup.h"
#include "stm32h7xx_hal.h"
#include "tasks/storage.h"
#include "timer.h"

void mgmt_hard_reset() {
    // Give storage the grace period to stop
    storage_pause(STORAGE_PAUSE_RESET);
    DELAY(HARD_RESET_GRACE_MS);

    // Invalidate backup and reset
    vTaskSuspendAll();
    backup_invalidate();
    NVIC_SystemReset();
}
