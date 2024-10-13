#include "nand_flash.h"

#include "fatfs/diskio.h"
#include "fatfs/ff.h"
#include "fifos.h"
#include "main.h"
#include "pb_create.h"
#include "rtc/rtc.h"
#include "stdio.h"
#include "stdlib.h"
#include "timer.h"

#define FNAME_LEN 64
#define HEADER_LEN 64

FATFS g_fs;
int g_nand_ready = 0;

// static char s_cfg_dir[FDIR_LEN] = NAND_MOUNT_POINT "/cfg";
// static char s_filename[FNAME_LEN] = "/dat_00.pb3";
// static char s_gpsfname[FNAME_LEN] = "/gps_00.pb3";
// static char s_statefname[FNAME_LEN] = "/fsl_00.pb3";
static char s_cfgfname[FNAME_LEN] = "/cfg.bin";
// static char s_logfname[FNAME_LEN] = "/log_00.txt";
static char s_prffname[FNAME_LEN] = "/prf_00.txt";

// static pb_byte_t s_sensor_buffer[SENSOR_BUF_LEN];
// static pb_byte_t s_gps_buffer[GPS_BUF_LEN];
// static pb_byte_t s_state_buffer[STATE_BUF_LEN];

static const char s_header[HEADER_LEN] = FIRMWARE_SPECIFIER "\n";

extern uint32_t mtp_file_idx;

Status nand_flash_open_binary_file(FIL* fp, const char* fname) {
    char new_path[256];  // Max path length
    snprintf(new_path, 256, NAND_MOUNT_POINT "%s", fname);

    // Create sensor data file
    if (f_open(fp, new_path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write the header
    UINT bw = 0;
    if (f_write(fp, s_header, strlen(s_header), &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check the correct number of bytes was written
    if (bw != strlen(s_header)) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (f_sync(fp) != 0) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_close_file(FIL* fp) {
    if (f_close(fp) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_write_binary_data(FIL* fp, uint8_t* data, size_t size) {
    // Check if data pointer is valid
    if (data == NULL) {
        return STATUS_ERROR;
    }

    // Check if the file pointer is valid
    if (fp == NULL || fp->obj.fs == NULL) {
        return STATUS_ERROR;
    }

    // Write the data to the file
    UINT bw;
    if (f_write(fp, data, size, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_dump_prf_stats(char stats[]) {
    FIL prffile;
    UINT bw = 0;

    // Create performance dump file
    if (f_open(&prffile, s_prffname, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write a timestamp
    char timestamp[32];
    int size = snprintf(timestamp, 32, "%lu ms:\n", (uint32_t)MILLIS());
    if (size > 0) {
        bw += f_write(&prffile, timestamp, size, &bw);
    }

    // Write the stats
    size = strlen(stats);
    bw += f_write(&prffile, stats, size, &bw);

    // Check that something was written
    if (bw == 0) {
        return STATUS_HARDWARE_ERROR;
    }

    // Close the file
    if (f_close(&prffile) != 0) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_init() {
    if (mt29f4g_init() != STATUS_OK) {
        PAL_LOGE("Failed to initialize NAND\n");
        return STATUS_HARDWARE_ERROR;
    }
    memset(&g_fs, 0, sizeof(g_fs));
    int status = f_mount(&g_fs, NAND_MOUNT_POINT, 1);
#ifdef NAND_ALLOW_REFORMAT
    if (status == FR_NO_FILESYSTEM) {
        PAL_LOGW("No file system found. Formatting...\n");
        // Fully erase the chip before formatting
        status = mt29f4g_erase_chip();
        if (status != STATUS_OK) {
            PAL_LOGE("Failed to erase NAND: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
        BYTE work[FF_MAX_SS];
        memset(work, 0, sizeof(work));
        MKFS_PARM format_opts;
        memset(&format_opts, 0, sizeof(format_opts));
        format_opts.fmt = FM_FAT32;

        status = f_mkfs(NAND_MOUNT_POINT, &format_opts, work, FF_MAX_SS);
        if (status != FR_OK) {
            PAL_LOGE("Failed to format NAND: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
        status = f_setlabel(NAND_MOUNT_POINT "/" NAND_LABEL);
        if (status != FR_OK) {
            PAL_LOGE("Failed to set label: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
        status = f_mount(&g_fs, NAND_MOUNT_POINT, 1);
    }
#endif
    if (status != FR_OK) {
        PAL_LOGE("Failed to mount NAND: %d\n", status);
        return STATUS_HARDWARE_ERROR;
    }
    PAL_LOGI("NAND mounted successfully!\n");

    // See the files in the root directory
    if (f_ls(NAND_MOUNT_POINT, 0) != 0) {
        PAL_LOGE("Failed to list files on flash\n");
        return STATUS_HARDWARE_ERROR;
    }

    // Get free space
    DWORD clusters;
    FATFS* fs = &g_fs;
    if (f_getfree(NAND_MOUNT_POINT, &clusters, &fs) != FR_OK) {
        PAL_LOGE("Failed to get nand filesystem space\n");
        return STATUS_HARDWARE_ERROR;
    }
    uint32_t fs_free = clusters * fs->csize * fs->ssize;
    static const char* prefixes[] = {"", "K", "M", "G"};
    for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
        if (fs_free >= (1 << 10 * i) - 1) {
            printf("Remaining space: %*lu%sB\n", 4 - (i != 0),
                   fs_free >> 10 * i, prefixes[i]);
            break;
        }
    }

    return STATUS_OK;
}

char** nand_flash_get_file_list(const char* path, size_t* num_files) {
    char new_path[256];  // Max path length
    snprintf(new_path, 256, NAND_MOUNT_POINT "%s", path);
    char** ret = NULL;

    // First pass: count the number of files
    *num_files = 0;
    DIR dir;
    if (f_opendir(&dir, new_path) != FR_OK) {
        return ret;
    }

    FILINFO info;
    while (true) {
        if (f_readdir(&dir, &info) != FR_OK) {
            f_closedir(&dir);
            return ret;
        }

        if (info.fname[0] == '\0') {
            break;
        }

        if (info.fattrib & AM_DIR) {
            continue;
        } else {
            *num_files += 1;
        }
    }

    if (num_files == 0) {
        return ret;
    }

    // Allocate space for the filenames
    ret = (char**)malloc(*num_files * sizeof(char*));
    *num_files = 0;

    // Second pass: copy the filenames
    if (f_opendir(&dir, new_path) != FR_OK) {
        free(ret);
        return NULL;
    }

    while (true) {
        if (f_readdir(&dir, &info) != FR_OK) {
            f_closedir(&dir);
            return ret;
        }

        if (info.fname[0] == '\0') {
            break;
        }

        if (info.fattrib & AM_DIR) {
            continue;
        } else {
            // Copy the filename to the list
            ret[*num_files] = (char*)malloc(strlen(info.fname) + 1);
            strcpy(ret[*num_files], info.fname);
            (*num_files)++;
        }
    }

    if (f_closedir(&dir) != 0) {
        return ret;
    }

    g_nand_ready = 1;
    return ret;
}

Status nand_flash_delete_file_list(char** file_list, size_t num_files) {
    if (file_list == NULL) {
        return STATUS_OK;
    }

    for (size_t i = 0; i < num_files; i++) {
        if (file_list[i] != NULL) free(file_list[i]);
    }
    free(file_list);

    return STATUS_OK;
}

Status nand_flash_reinit() {
    g_nand_ready = 0;
    g_nand_ready = 1;
    return STATUS_OK;
}

Status nand_flash_deinit() {
    g_nand_ready = 0;
    return STATUS_OK;
}

Status nand_flash_flush(FIL* fp) {
    if (f_sync(fp) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_load_board_config(BoardConfig* board_config) {
    FIL cfgfile;

    if (f_open(&cfgfile, s_cfgfname, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    UINT br;
    if (f_read(&cfgfile, board_config, sizeof(board_config), &br) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    if (br != sizeof(board_config)) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_store_board_config(BoardConfig* board_config) {
    FIL cfgfile;

    if (f_open(&cfgfile, s_cfgfname, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    UINT bw;
    if (f_write(&cfgfile, board_config, sizeof(board_config), &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    if (bw != sizeof(board_config)) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

int f_ls(const char* path, int depth) {
    DIR dir;
    if (f_opendir(&dir, path) != FR_OK) {
        return STATUS_ERROR;
    }

    FILINFO info;
    while (true) {
        if (f_readdir(&dir, &info) != FR_OK) {
            f_closedir(&dir);
            return STATUS_ERROR;
        }

        if (info.fname[0] == '\0') {
            break;
        }

        if (depth > 0) {
            for (int i = 0; i < depth - 1; i++) {
                printf(" ");
            }
            printf("|- ");
        }

        switch (info.fattrib & AM_DIR) {
            case 0:
                printf("reg ");
                break;
            case AM_DIR:
                printf("dir ");
                break;
        }

        static const char* prefixes[] = {"", "K", "M", "G"};
        for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
            if (info.fsize >= (1 << 10 * i) - 1) {
                printf("%*lu%sB ", 4 - (i != 0), (uint32_t)info.fsize >> 10 * i,
                       prefixes[i]);
                break;
            }
        }

        printf("%s\n", info.fname);

        // Recurse into directories
        if (info.fattrib & AM_DIR) {
            char new_path[256];
            snprintf(new_path, 256, "%s/%s", path, info.fname);
            f_ls(new_path, depth + 1);
        }
    }

    if (f_closedir(&dir) != 0) {
        return STATUS_ERROR;
    }

    return 0;
}

void nand_flash_capacity(uint32_t* block_count, uint16_t* block_size) {
    disk_ioctl(1, GET_SECTOR_COUNT, block_count);
    disk_ioctl(1, GET_SECTOR_SIZE, block_size);
}

Status nand_flash_raw_write(const BYTE* buff, LBA_t sector, UINT count) {
    if (disk_write(1, buff, sector, count) == RES_OK) {
        return STATUS_OK;
    }
    return STATUS_ERROR;
}

Status nand_flash_raw_read(BYTE* buff, LBA_t sector, UINT count) {
    if (disk_read(1, buff, sector, count) == RES_OK) {
        return STATUS_OK;
    }
    return STATUS_ERROR;
}

Status nand_flash_mkdir(const char* fname) {
    char new_path[256];  // Max path length
    snprintf(new_path, 256, NAND_MOUNT_POINT "%s", fname);

    FRESULT res = f_mkdir(new_path);
    if (res == FR_OK || res || FR_EXIST) {
        return STATUS_OK;
    }
    return STATUS_ERROR;
}