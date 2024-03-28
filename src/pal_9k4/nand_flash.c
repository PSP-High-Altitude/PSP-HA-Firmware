#include "nand_flash.h"

#include "littlefs/lfs.h"
#include "main.h"
#include "pb_create.h"
#include "stdio.h"
#include "timer.h"
#include "usbd_mtp_if.h"

#define FNAME_LEN 16
#define FDIR_LEN 32
#define HEADER_LEN 64

extern uint8_t usb_mode;

static char s_fltdata_dir[FDIR_LEN] = "";
static char s_filename[FNAME_LEN + FDIR_LEN] = "dat_00.pb3";
static char s_gpsfname[FNAME_LEN + FDIR_LEN] = "gps_00.pb3";
static char s_statefname[FNAME_LEN + FDIR_LEN] = "fsl_00.pb3";
static char s_prffname[FNAME_LEN + FDIR_LEN] = "prf_00.txt";

static struct lfs_file s_datfile;
static struct lfs_file s_gpsfile;
static struct lfs_file s_statefile;
static uint8_t s_datfile_open = 0;
static uint8_t s_gpsfile_open = 0;
static uint8_t s_statefile_open = 0;

lfs_t g_fs = {0};
struct lfs_config* g_lfs_cfg;

// static pb_byte_t s_sensor_buffer[SENSOR_BUF_LEN];
// static pb_byte_t s_gps_buffer[GPS_BUF_LEN];
// static pb_byte_t s_state_buffer[STATE_BUF_LEN];

static const char s_header[HEADER_LEN] = FIRMWARE_SPECIFIER;

extern char mtp_file_names[128][LFS_NAME_MAX + 1];  // 128 files
extern uint32_t mtp_file_idx;

static Status nand_flash_create_sensor_file() {
    // Create sensor data file
    if (lfs_file_open(&g_fs, &s_datfile, s_filename,
                      LFS_O_CREAT | LFS_O_WRONLY) != LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    s_datfile_open = 1;

    // Write the header
    lfs_ssize_t bw = lfs_file_write(&g_fs, &s_datfile, s_header, HEADER_LEN);

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (lfs_file_sync(&g_fs, &s_datfile) != LFS_ERR_OK) {
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
    if (lfs_file_open(&g_fs, &s_gpsfile, s_gpsfname,
                      LFS_O_CREAT | LFS_O_WRONLY) != LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    s_gpsfile_open = 1;

    // Write the header
    lfs_ssize_t bw = lfs_file_write(&g_fs, &s_gpsfile, s_header, HEADER_LEN);

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (lfs_file_sync(&g_fs, &s_gpsfile) != LFS_ERR_OK) {
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
    if (lfs_file_open(&g_fs, &s_statefile, s_statefname,
                      LFS_O_CREAT | LFS_O_WRONLY) != LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    s_statefile_open = 1;

    // Write the header
    lfs_ssize_t bw = lfs_file_write(&g_fs, &s_statefile, s_header, HEADER_LEN);

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (lfs_file_sync(&g_fs, &s_statefile) != LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check that the frame will at least nominally fit in our buffer
    if (STATE_BUF_LEN < sizeof(StateFrame)) {
        return STATUS_DATA_ERROR;
    }

    return STATUS_OK;
}

static Status nand_flash_open_files() {
    // Traverse the directory to find the oldest and newest files
    lfs_dir_t dir;
    int err = lfs_dir_open(&g_fs, &dir, s_fltdata_dir);
    if (err) {
        return STATUS_ERROR;
    }

    int32_t min_file_suffix = 100;
    int32_t max_file_suffix = -1;

    struct lfs_info info;
    while (true) {
        int res = lfs_dir_read(&g_fs, &dir, &info);
        if (res < 0) {
            lfs_dir_close(&g_fs, &dir);
            return STATUS_ERROR;
        }

        if (res == 0) {
            break;
        }

        if (info.type != LFS_TYPE_REG) {
            continue;
        }

        if (strncmp(info.name, "dat_", 4) == 0) {
            int32_t suffix = atoi(info.name + 4);
            if (suffix > max_file_suffix) {
                max_file_suffix = suffix;
            }
            if (suffix < min_file_suffix) {
                min_file_suffix = suffix;
            }
        }
    }

    lfs_dir_close(&g_fs, &dir);

    // Remove files if there are more than NAND_MAX_FLIGHTS
    while (max_file_suffix - min_file_suffix > NAND_MAX_FLIGHTS - 1) {
        snprintf(s_filename, FNAME_LEN + FDIR_LEN, "%sdat_%02ld.pb3",
                 s_fltdata_dir, min_file_suffix);
        snprintf(s_gpsfname, FNAME_LEN + FDIR_LEN, "%sgps_%02ld.pb3",
                 s_fltdata_dir, min_file_suffix);
        snprintf(s_statefname, FNAME_LEN + FDIR_LEN, "%sfsl_%02ld.pb3",
                 s_fltdata_dir, min_file_suffix);
        snprintf(s_prffname, FNAME_LEN + FDIR_LEN, "%sprf_%02ld.txt",
                 s_fltdata_dir, min_file_suffix);
        lfs_remove(&g_fs, s_filename);
        lfs_remove(&g_fs, s_gpsfname);
        lfs_remove(&g_fs, s_statefname);
        lfs_remove(&g_fs, s_prffname);
        min_file_suffix++;
    }

    // Roll around if we reach 99
    if (max_file_suffix > 99) {
        max_file_suffix = 0;
    }

    // Create new file names
    snprintf(s_filename, FNAME_LEN + FDIR_LEN, "%sdat_%02ld.pb3", s_fltdata_dir,
             max_file_suffix + 1);
    snprintf(s_gpsfname, FNAME_LEN + FDIR_LEN, "%sgps_%02ld.pb3", s_fltdata_dir,
             max_file_suffix + 1);
    snprintf(s_statefname, FNAME_LEN + FDIR_LEN, "%sfsl_%02ld.pb3",
             s_fltdata_dir, max_file_suffix + 1);
    snprintf(s_prffname, FNAME_LEN + FDIR_LEN, "%sprf_%02ld.txt", s_fltdata_dir,
             max_file_suffix + 1);

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

    return STATUS_OK;
}

Status nand_flash_write_sensor_data(SensorFrame* frame) {
    size_t buf_size = 0;
    pb_byte_t* buf = create_sensor_buffer(frame, &buf_size);

    if (buf == NULL) {
        return STATUS_ERROR;
    }

    if (!s_datfile_open) {
        return STATUS_ERROR;
    }

    if (lfs_file_write(&g_fs, &s_datfile, (uint8_t*)buf, buf_size) !=
        LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_write_gps_data(GpsFrame* frame) {
    size_t buf_size;
    pb_byte_t* buf = create_gps_buffer(frame, &buf_size);

    if (buf == NULL) {
        return STATUS_ERROR;
    }

    if (!s_gpsfile_open) {
        return STATUS_ERROR;
    }

    if (lfs_file_write(&g_fs, &s_gpsfile, (uint8_t*)buf, buf_size) !=
        LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_write_state_data(StateFrame* frame) {
    size_t buf_size;
    pb_byte_t* buf = create_state_buffer(frame, &buf_size);

    if (buf == NULL) {
        return STATUS_ERROR;
    }

    if (!s_statefile_open) {
        return STATUS_ERROR;
    }

    if (lfs_file_write(&g_fs, &s_statefile, (uint8_t*)buf, buf_size) !=
        LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_dump_prf_stats(char stats[]) {
    lfs_file_t prffile;
    size_t bw = 0;

    // Create performance dump file
    if (lfs_file_open(&g_fs, &prffile, s_prffname,
                      LFS_O_APPEND | LFS_O_CREAT | LFS_O_WRONLY) !=
        LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write a timestamp
    char timestamp[32];
    int size = snprintf(timestamp, 32, "%lu ms: ", (uint32_t)MILLIS());
    if (size > 0) {
        bw += lfs_file_write(&g_fs, &prffile, timestamp, size);
    }

    // Write the stats
    size = strlen(stats);
    bw += lfs_file_write(&g_fs, &prffile, stats, size);

    // Check that something was written
    if (bw == 0) {
        return STATUS_HARDWARE_ERROR;
    }

    // Close the file
    if (lfs_file_close(&g_fs, &prffile) != LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_init() {
    if (mt29f4g_init() != STATUS_OK) {
        printf("Failed to initialize NAND hardware\n");
        return STATUS_HARDWARE_ERROR;
    }

    g_lfs_cfg = mt29f4g_get_lfs_config();
    int status = lfs_mount(&g_fs, g_lfs_cfg);
    if (status != LFS_ERR_OK) {
#ifdef NAND_ALLOW_REFORMAT
        if (status != LFS_ERR_IO) {
            printf("No file system found. Formatting...\n");
            // Fully erase the chip before formatting
            status = mt29f4g_erase_chip();
            if (status != STATUS_OK) {
                printf("Failed to erase NAND: %d\n", status);
                return STATUS_HARDWARE_ERROR;
            }
            status = lfs_format(&g_fs, g_lfs_cfg);
            if (status != LFS_ERR_OK) {
                printf("Failed to format NAND: %d\n", status);
                return STATUS_HARDWARE_ERROR;
            }
            status = lfs_mount(&g_fs, g_lfs_cfg);
            if (status != LFS_ERR_OK) {
                printf("Failed to mount NAND: %d\n", status);
                return STATUS_HARDWARE_ERROR;
            }
        } else {
            printf("Failed to mount NAND: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
#else
        return STATUS_HARDWARE_ERROR;
#endif
    }

    if (usb_mode == 1) {
        // Open the files
        Status open_file_stat = nand_flash_open_files();
        if (open_file_stat != STATUS_OK) {
            return open_file_stat;
        }
    }

    // See the files in the root directory
    if (lfs_ls(&g_fs, "/") != 0) {
        printf("Failed to list files on flash\n");
        return STATUS_HARDWARE_ERROR;
    }

    // See the space remaining
    lfs_ssize_t fs_size = lfs_fs_size(&g_fs);
    if (fs_size < 0) {
        printf("Failed to get file system space\n");
        return STATUS_HARDWARE_ERROR;
    } else {
        lfs_size_t fs_free =
            (g_lfs_cfg->block_count * g_lfs_cfg->block_size) - fs_size;
        static const char *prefixes[] = {"", "K", "M", "G"};
        for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
            if (fs_free >= (1 << 10 * i) - 1) {
                printf("Remaining space: %*lu%sB\n", 4 - (i != 0),
                       fs_free >> 10 * i, prefixes[i]);
                break;
            }
        }
    }

    return STATUS_OK;
}

Status nand_flash_reinit() {
    if (!s_datfile_open) {
        if (lfs_file_open(&g_fs, &s_datfile, s_filename,
                          LFS_O_APPEND | LFS_O_WRONLY) != LFS_ERR_OK) {
            return STATUS_HARDWARE_ERROR;
            s_datfile_open = 0;
        }
        s_datfile_open = 1;
    }
    if (!s_gpsfile_open) {
        if (lfs_file_open(&g_fs, &s_gpsfile, s_gpsfname,
                          LFS_O_APPEND | LFS_O_WRONLY) != LFS_ERR_OK) {
            return STATUS_HARDWARE_ERROR;
            s_gpsfile_open = 0;
        }
        s_gpsfile_open = 1;
    }
    if (!s_statefile_open) {
        if (lfs_file_open(&g_fs, &s_statefile, s_statefname,
                          LFS_O_APPEND | LFS_O_WRONLY) != LFS_ERR_OK) {
            return STATUS_HARDWARE_ERROR;
            s_statefile_open = 0;
        }
        s_statefile_open = 1;
    }

    return STATUS_OK;
}

Status nand_flash_deinit() {
    if (s_datfile_open) {
        if (lfs_file_close(&g_fs, &s_datfile) != LFS_ERR_OK) {
            return STATUS_HARDWARE_ERROR;
        }
        s_datfile_open = 0;
    }
    if (s_gpsfile_open) {
        if (lfs_file_close(&g_fs, &s_gpsfile) != LFS_ERR_OK) {
            return STATUS_HARDWARE_ERROR;
        }
        s_gpsfile_open = 0;
    }
    if (s_statefile_open) {
        if (lfs_file_close(&g_fs, &s_statefile) != LFS_ERR_OK) {
            return STATUS_HARDWARE_ERROR;
        }
        s_statefile_open = 0;
    }

    return STATUS_OK;
}

Status nand_flash_flush() {
    if (s_datfile_open) {
        if (lfs_file_sync(&g_fs, &s_datfile) != LFS_ERR_OK) {
            return STATUS_HARDWARE_ERROR;
        }
    }
    if (s_gpsfile_open) {
        if (lfs_file_sync(&g_fs, &s_gpsfile) != LFS_ERR_OK) {
            return STATUS_HARDWARE_ERROR;
        }
    }
    if (s_statefile_open) {
        if (lfs_file_sync(&g_fs, &s_statefile) != LFS_ERR_OK) {
            return STATUS_HARDWARE_ERROR;
        }
    }

    return STATUS_OK;
}

// From user geky on github
int lfs_ls(lfs_t* lfs, const char* path) {
    lfs_dir_t dir;
    int err = lfs_dir_open(lfs, &dir, path);
    if (err) {
        return err;
    }

    struct lfs_info info;
    while (true) {
        int res = lfs_dir_read(lfs, &dir, &info);
        if (res < 0) {
            lfs_dir_close(lfs, &dir);
            return res;
        }

        if (res == 0) {
            break;
        }

        switch (info.type) {
            case LFS_TYPE_REG:
                printf("reg ");
                break;
            case LFS_TYPE_DIR:
                printf("dir ");
                break;
            default:
                printf("?   ");
                break;
        }

        static const char* prefixes[] = {"", "K", "M", "G"};
        for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
            if (info.size >= (1 << 10 * i) - 1) {
                printf("%*lu%sB ", 4 - (i != 0), info.size >> 10 * i,
                       prefixes[i]);
                break;
            }
        }

        printf("%s\n", info.name);
    }

    err = lfs_dir_close(lfs, &dir);
    if (err) {
        return err;
    }

    return 0;
}