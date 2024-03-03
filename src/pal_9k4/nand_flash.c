#include "nand_flash.h"

#include "littlefs/lfs.h"
#include "stdio.h"

static lfs_t s_fs;
struct lfs_config *lfs_cfg;
#define NAND_MOUNT_POINT "/NAND"

Status nand_flash_init() {
    lfs_cfg = mt29f2g_get_lfs_config();
    int status = lfs_mount(&s_fs, lfs_cfg);
    if (status != LFS_ERR_OK) {
        if (status != LFS_ERR_IO) {
            printf("No file system found. Formatting...\n");
            status = lfs_format(&s_fs, lfs_cfg);
            if (status != LFS_ERR_OK) {
                printf("Failed to format NAND: %d\n", status);
                return STATUS_HARDWARE_ERROR;
            }
            status = lfs_mount(&s_fs, lfs_cfg);
            if (status != LFS_ERR_OK) {
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