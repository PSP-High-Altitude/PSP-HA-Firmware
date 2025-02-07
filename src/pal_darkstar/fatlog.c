#include "fatlog.h"

#include "main.h"
#include "rtc/rtc.h"
#include "stdio.h"
#include "stdlib.h"
#include "timer.h"

#define MOUNT_POINT "/MMC"
#define VOLUME_LABEL "Darkstar"

#define FNAME_LEN 64
#define HEADER_LEN 64
#define MAXPATH_LEN 256

RAM_D2 static FATFS s_fs;

static void snfmtspace(char* str, size_t str_size, uint64_t bytes) {
    static const char* prefixes[] = {"", "K", "M", "G"};
    for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
        if (bytes >= (1 << 10 * i) - 1) {
            snprintf(str, str_size, "%*lu%sB", bytes >> 10 * i, prefixes[i]);
            break;
        }
    }
}

Status fatlog_open_file_for_write(FIL* fp, const char* fname) {
    char new_path[MAXPATH_LEN];
    snprintf(new_path, MAXPATH_LEN, MOUNT_POINT "%s", fname);

    if (f_open(fp, new_path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status fatlog_open_file_for_read(FIL* fp, const char* fname) {
    char new_path[MAXPATH_LEN];
    snprintf(new_path, MAXPATH_LEN, MOUNT_POINT "%s", fname);

    if (f_open(fp, new_path, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status fatlog_close_file(FIL* fp) {
    if (f_close(fp) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status fatlog_write_data(FIL* fp, uint8_t* data, size_t size) {
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

    if (bw != size) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status fatlog_read_data(FIL* fp, uint8_t* data, size_t size) {
    // Check if data pointer is valid
    if (data == NULL) {
        return STATUS_ERROR;
    }

    // Check if the file pointer is valid
    if (fp == NULL || fp->obj.fs == NULL) {
        return STATUS_ERROR;
    }

    // Read the data from the file
    UINT br;
    if (f_read(fp, data, size, &br) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    if (br != size) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status fatlog_init() {
    FRESULT res = f_mount(&s_fs, MOUNT_POINT, 1);
    if (res != FR_OK) {
#ifdef NAND_ALLOW_REFORMAT
        if (res == FR_NO_FILESYSTEM) {
            BYTE work[FF_MAX_SS];
            memset(work, 0, sizeof(work));
            MKFS_PARM format_opts;
            memset(&format_opts, 0, sizeof(format_opts));
            format_opts.fmt = FM_FAT32;

            res = f_mkfs(MOUNT_POINT, &format_opts, work, FF_MAX_SS);
            if (res != FR_OK) {
                return STATUS_HARDWARE_ERROR;
            }
            res = f_setlabel(MOUNT_POINT "/" VOLUME_LABEL);
            if (res != FR_OK) {
                return STATUS_HARDWARE_ERROR;
            }
            res = f_mount(&s_fs, MOUNT_POINT, 1);
        }
#endif
        return STATUS_HARDWARE_ERROR;
    }

    // Get free space
    uint64_t total_bytes;
    uint64_t free_bytes;
    fatlog_space(&total_bytes, &free_bytes);

    char total_space_str[16];
    char free_space_str[16];
    snfmtspace(total_space_str, 16, total_bytes);
    snfmtspace(free_space_str, 16, free_bytes);
    PAL_LOGI("Remaining space: %s/%s\n", total_space_str, free_space_str);

    return STATUS_OK;
}

char** fatlog_get_file_list(const char* path, size_t* num_files) {
    char new_path[MAXPATH_LEN];
    snprintf(new_path, MAXPATH_LEN, MOUNT_POINT "%s", path);
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

    return ret;
}

Status fatlog_delete_file_list(char** file_list, size_t num_files) {
    if (file_list == NULL) {
        return STATUS_OK;
    }

    for (size_t i = 0; i < num_files; i++) {
        if (file_list[i] != NULL) free(file_list[i]);
    }
    free(file_list);

    return STATUS_OK;
}

Status fatlog_reinit() {
    if (f_mount(&s_fs, MOUNT_POINT, 1) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status fatlog_deinit() {
    if (f_unmount(MOUNT_POINT) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status fatlog_flush(FIL* fp) {
    if (f_sync(fp) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status fatlog_space(uint64_t* total_bytes, uint64_t* free_bytes) {
    DWORD fre_clust;

    FATFS* fs = &s_fs;

    FRESULT res = f_getfree(MOUNT_POINT, &fre_clust, &fs);
    if (res) {
        PAL_LOGE("Failed to get filesystem space\n");
        return STATUS_HARDWARE_ERROR;
    }

    *total_bytes = (fs->n_fatent - 2) * fs->csize * 512;
    *free_bytes = fre_clust * fs->csize * 512;

    return STATUS_OK;
}

Status fatlog_mkdir(const char* fname) {
    char new_path[256];
    snprintf(new_path, 256, MOUNT_POINT "%s", fname);

    FRESULT res = f_mkdir(new_path);
    if (res == FR_OK || res || FR_EXIST) {
        return STATUS_OK;
    }
    return STATUS_ERROR;
}
