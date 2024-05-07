#include "nand_flash.h"

#include "fatfs/ff.h"
#include "stdio.h"

static FATFS s_fs;
#define NAND_MOUNT_POINT "/NAND"

Status nand_flash_init() {
    FRESULT status = f_mount(&s_fs, NAND_MOUNT_POINT, 1);
    if (status != FR_OK) {
        if (status == FR_NO_FILESYSTEM) {
            printf("No file system found. Formatting...\n");
            BYTE work[FF_MAX_SS];
            status = f_mkfs(NAND_MOUNT_POINT, 0, work, sizeof work);
            if (status != FR_OK) {
                printf("Failed to format NAND: %d\n", status);
                return STATUS_HARDWARE_ERROR;
            }
            status = f_mount(&s_fs, NAND_MOUNT_POINT, 1);
            if (status != FR_OK) {
                printf("Failed to mount NAND: %d\n", status);
                return STATUS_HARDWARE_ERROR;
            }
        } else {
            printf("Failed to mount NAND: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
    }
    return STATUS_OK;
}