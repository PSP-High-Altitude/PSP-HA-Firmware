#include "nand_flash.h"

#include "fatfs/ff.h"
#include "main.h"
#include "pb_create.h"
#include "rtc.h"
#include "stdio.h"
#include "timer.h"
#include "usbd_mtp_if.h"

#define FNAME_LEN 16
#define FDIR_LEN 32
#define HEADER_LEN 64

static FATFS g_fs;

extern uint8_t mtp_mode;

static char s_fltdata_dir[FDIR_LEN] = NAND_MOUNT_POINT "/data";
// static char s_cfg_dir[FDIR_LEN] = NAND_MOUNT_POINT "/cfg";
static char s_filename[FNAME_LEN + FDIR_LEN] = "/dat_00.pb3";
static char s_gpsfname[FNAME_LEN + FDIR_LEN] = "/gps_00.pb3";
static char s_statefname[FNAME_LEN + FDIR_LEN] = "/fsl_00.pb3";
static char s_cfgfname[FNAME_LEN + FDIR_LEN] = "/cfg.bin";
static char s_logfname[FNAME_LEN + FDIR_LEN] = "/log_00.txt";
static char s_prffname[FNAME_LEN + FDIR_LEN] = "/prf_00.txt";

static FIL s_datfile;
static FIL s_gpsfile;
static FIL s_statefile;
static FIL s_logfile;
static uint8_t s_datfile_open = 0;
static uint8_t s_gpsfile_open = 0;
static uint8_t s_statefile_open = 0;
static uint8_t s_logfile_open = 0;

// static pb_byte_t s_sensor_buffer[SENSOR_BUF_LEN];
// static pb_byte_t s_gps_buffer[GPS_BUF_LEN];
// static pb_byte_t s_state_buffer[STATE_BUF_LEN];

static const char s_header[HEADER_LEN] = FIRMWARE_SPECIFIER "\n";

extern uint32_t mtp_file_idx;

static Status nand_flash_create_sensor_file() {
    // Create sensor data file
    if (f_open(&s_datfile, s_filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    s_datfile_open = 1;

    // Write the header
    UINT bw = 0;
    if (f_write(&s_datfile, s_header, strlen(s_header), &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check the correct number of bytes was written
    if (bw != strlen(s_header)) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (f_sync(&s_datfile) != 0) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check that the frame will at least nominally fit in our buffer
    if (SENSOR_BUF_LEN < sizeof(SensorFrame)) {
        return STATUS_DATA_ERROR;
    }

    return STATUS_OK;
}

static Status nand_flash_create_gps_file() {
    // Create gps data file
    if (f_open(&s_gpsfile, s_gpsfname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    s_gpsfile_open = 1;

    // Write the header
    UINT bw = 0;
    if (f_write(&s_gpsfile, s_header, strlen(s_header), &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check the correct number of bytes was written
    if (bw != strlen(s_header)) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (f_sync(&s_gpsfile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check that the frame will at least nominally fit in our buffer
    if (SENSOR_BUF_LEN < sizeof(GpsFrame)) {
        return STATUS_DATA_ERROR;
    }

    return STATUS_OK;
}

static Status nand_flash_create_state_file() {
    // Create state data file
    if (f_open(&s_statefile, s_statefname, FA_CREATE_ALWAYS | FA_WRITE) !=
        FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    s_statefile_open = 1;

    // Write the header
    UINT bw = 0;
    if (f_write(&s_statefile, s_header, strlen(s_header), &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check the correct number of bytes was written
    if (bw != strlen(s_header)) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (f_sync(&s_statefile) != 0) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check that the frame will at least nominally fit in our buffer
    if (STATE_BUF_LEN < sizeof(StateFrame)) {
        return STATUS_DATA_ERROR;
    }

    return STATUS_OK;
}

static Status nand_flash_create_log_file() {
    // Create log file
    if (f_open(&s_logfile, s_logfname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    s_logfile_open = 1;

    // Write the header
    UINT bw = 0;
    if (f_write(&s_logfile, s_header, strlen(s_header), &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check the correct number of bytes was written
    if (bw != strlen(s_header)) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (f_sync(&s_logfile) != 0) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

static Status nand_flash_open_files() {
    // Traverse the directory to find the oldest and newest files
    DIR dir;
    if (f_opendir(&dir, s_fltdata_dir) != FR_OK) {
        // If we can't open the directory, make it and try again
        if (f_mkdir(s_fltdata_dir) != FR_OK) {
            return STATUS_ERROR;
        }
        if (f_opendir(&dir, s_fltdata_dir) != FR_OK) {
            return STATUS_ERROR;
        }
    }

    int32_t min_file_suffix = 100;
    int32_t max_file_suffix = -1;

    FILINFO info;
    while (true) {
        if (f_readdir(&dir, &info) != FR_OK) {
            f_closedir(&dir);
            return STATUS_ERROR;
        }
        if (info.fname[0] == '\0') {
            break;
        }

        if (info.fattrib & AM_DIR) {
            continue;
        }

        if (strncmp(info.fname, "dat_", 4) == 0) {
            int32_t suffix = atoi(info.fname + 4);
            if (suffix > max_file_suffix) {
                max_file_suffix = suffix;
            }
            if (suffix < min_file_suffix) {
                min_file_suffix = suffix;
            }
        }
    }

    f_closedir(&dir);

    // Remove files if there are more than NAND_MAX_FLIGHTS
    while (max_file_suffix - min_file_suffix > NAND_MAX_FLIGHTS - 1) {
        snprintf(s_filename, FNAME_LEN + FDIR_LEN, "%s/dat_%02ld.pb3",
                 s_fltdata_dir, min_file_suffix);
        snprintf(s_gpsfname, FNAME_LEN + FDIR_LEN, "%s/gps_%02ld.pb3",
                 s_fltdata_dir, min_file_suffix);
        snprintf(s_statefname, FNAME_LEN + FDIR_LEN, "%s/fsl_%02ld.pb3",
                 s_fltdata_dir, min_file_suffix);
        snprintf(s_prffname, FNAME_LEN + FDIR_LEN, "%s/prf_%02ld.txt",
                 s_fltdata_dir, min_file_suffix);
        snprintf(s_logfname, FNAME_LEN + FDIR_LEN, "%s/log_%02ld.txt",
                 s_fltdata_dir, min_file_suffix);
        f_unlink(s_filename);
        f_unlink(s_gpsfname);
        f_unlink(s_statefname);
        f_unlink(s_prffname);
        f_unlink(s_logfname);
        min_file_suffix++;
    }

    // Roll around if we reach 99
    if (max_file_suffix > 99) {
        max_file_suffix = 0;
    }

    // Create new file names
    snprintf(s_filename, FNAME_LEN + FDIR_LEN, "%s/dat_%02ld.pb3",
             s_fltdata_dir, max_file_suffix + 1);
    snprintf(s_gpsfname, FNAME_LEN + FDIR_LEN, "%s/gps_%02ld.pb3",
             s_fltdata_dir, max_file_suffix + 1);
    snprintf(s_statefname, FNAME_LEN + FDIR_LEN, "%s/fsl_%02ld.pb3",
             s_fltdata_dir, max_file_suffix + 1);
    snprintf(s_prffname, FNAME_LEN + FDIR_LEN, "%s/prf_%02ld.txt",
             s_fltdata_dir, max_file_suffix + 1);
    snprintf(s_logfname, FNAME_LEN + FDIR_LEN, "%s/log_%02ld.txt",
             s_fltdata_dir, max_file_suffix + 1);

    // Initialize the sensor file and stream
    Status sensor_status = nand_flash_create_sensor_file();
    if (sensor_status != STATUS_OK) {
        return sensor_status;
    }

    // Initialize the GPS file and stream
    Status gps_status = nand_flash_create_gps_file();
    if (gps_status != STATUS_OK) {
        return gps_status;
    }

    // Initialize the flight state file and stream
    Status state_status = nand_flash_create_state_file();
    if (state_status != STATUS_OK) {
        return state_status;
    }

    // Initialize the log file
    Status log_status = nand_flash_create_log_file();
    if (log_status != STATUS_OK) {
        return log_status;
    }

    return STATUS_OK;
}

Status nand_flash_write_sensor_data(pb_byte_t* frame, size_t size) {
    if (frame == NULL) {
        return STATUS_ERROR;
    }

    if (!s_datfile_open) {
        return STATUS_ERROR;
    }

    UINT bw;
    if (f_write(&s_datfile, (uint8_t*)frame, size, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_write_gps_data(pb_byte_t* frame, size_t size) {
    if (frame == NULL) {
        return STATUS_ERROR;
    }

    if (!s_gpsfile_open) {
        return STATUS_ERROR;
    }

    UINT bw;
    if (f_write(&s_gpsfile, (uint8_t*)frame, size, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_write_state_data(pb_byte_t* frame, size_t size) {
    if (frame == NULL) {
        return STATUS_ERROR;
    }

    if (!s_statefile_open) {
        return STATUS_ERROR;
    }

    UINT bw;
    if (f_write(&s_statefile, (uint8_t*)frame, size, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

int nand_flash_write_log(const char* log, size_t size) {
    if (log == NULL) {
        return -1;
    }

    if (!s_logfile_open) {
        return -1;
    }

    RTCDateTime dt = rtc_get_datetime();

    // Determine size of new buffer
    int new_size =
        snprintf(NULL, 0, "%04ld-%02ld-%02ld %02ld:%02ld:%02ld     %.*s",
                 dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, size,
                 log) +
        1;

    char* new_buf = malloc(new_size);

    snprintf(new_buf, new_size, "%04ld-%02ld-%02ld %02ld:%02ld:%02ld     %.*s",
             dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, size,
             log);

    UINT bw;
    if (f_write(&s_logfile, (const uint8_t*)new_buf, new_size - 1, &bw) !=
        FR_OK) {
        free(new_buf);
        return -1;
    }

    free(new_buf);
    return bw;
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
    memset(&g_fs, 0, sizeof(g_fs));
    int status = f_mount(&g_fs, NAND_MOUNT_POINT, 1);
#ifdef NAND_ALLOW_REFORMAT
    if (status == FR_NO_FILESYSTEM) {
        printf("No file system found. Formatting...\n");
        // Fully erase the chip before formatting
        status = mt29f4g_erase_chip();
        if (status != STATUS_OK) {
            printf("Failed to erase NAND: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
        BYTE work[FF_MAX_SS];
        memset(work, 0, sizeof(work));
        MKFS_PARM format_opts;
        memset(&format_opts, 0, sizeof(format_opts));
        format_opts.fmt = FM_FAT32;

        status = f_mkfs(NAND_MOUNT_POINT, &format_opts, work, FF_MAX_SS);
        if (status != FR_OK) {
            printf("Failed to format NAND: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
        status = f_mount(&g_fs, NAND_MOUNT_POINT, 1);
    }
#endif
    if (status != FR_OK) {
        printf("Failed to mount NAND: %d\n", status);
        return STATUS_HARDWARE_ERROR;
    }
    printf("NAND mounted successfully!\n");

    if (mtp_mode == 0) {
        // Open the files
        Status open_file_stat = nand_flash_open_files();
        if (open_file_stat != STATUS_OK) {
            printf("Failed to open files on flash\n");
            return open_file_stat;
        }
    }

    // See the files in the root directory
    if (f_ls(NAND_MOUNT_POINT, 0) != 0) {
        printf("Failed to list files on flash\n");
        return STATUS_HARDWARE_ERROR;
    }

    // Get free space
    DWORD clusters;
    FATFS* fs = &g_fs;
    if (f_getfree(NAND_MOUNT_POINT, &clusters, &fs) != FR_OK) {
        printf("Failed to get nand filesystem space\n");
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

Status nand_flash_reinit() {
    if (!s_datfile_open) {
        if (f_open(&s_datfile, s_filename, FA_CREATE_ALWAYS | FA_WRITE) !=
            FR_OK) {
            return STATUS_HARDWARE_ERROR;
            s_datfile_open = 0;
        }
        s_datfile_open = 1;
    }
    if (!s_gpsfile_open) {
        if (f_open(&s_gpsfile, s_gpsfname, FA_CREATE_ALWAYS | FA_WRITE) !=
            FR_OK) {
            return STATUS_HARDWARE_ERROR;
            s_gpsfile_open = 0;
        }
        s_gpsfile_open = 1;
    }
    if (!s_statefile_open) {
        if (f_open(&s_statefile, s_statefname, FA_CREATE_ALWAYS | FA_WRITE) !=
            FR_OK) {
            return STATUS_HARDWARE_ERROR;
            s_statefile_open = 0;
        }
        s_statefile_open = 1;
    }
    if (!s_logfile_open) {
        if (f_open(&s_logfile, s_logfname, FA_CREATE_ALWAYS | FA_WRITE) !=
            FR_OK) {
            return STATUS_HARDWARE_ERROR;
            s_logfile_open = 0;
        }
        s_logfile_open = 1;
    }

    return STATUS_OK;
}

Status nand_flash_deinit() {
    if (s_datfile_open) {
        if (f_close(&s_datfile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
        s_datfile_open = 0;
    }
    if (s_gpsfile_open) {
        if (f_close(&s_gpsfile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
        s_gpsfile_open = 0;
    }
    if (s_statefile_open) {
        if (f_close(&s_statefile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
        s_statefile_open = 0;
    }
    if (s_logfile_open) {
        if (f_close(&s_logfile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
        s_logfile_open = 0;
    }

    return STATUS_OK;
}

Status nand_flash_flush() {
    if (s_datfile_open) {
        if (f_sync(&s_datfile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
    }
    if (s_gpsfile_open) {
        if (f_sync(&s_gpsfile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
    }
    if (s_statefile_open) {
        if (f_sync(&s_statefile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
    }
    if (s_logfile_open) {
        if (f_sync(&s_logfile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
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