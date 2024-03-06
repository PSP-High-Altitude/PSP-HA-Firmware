#include "nand_flash.h"

#include "littlefs/lfs.h"
#include "pb_encode.h"
#include "stdio.h"
#include "timer.h"

static lfs_t s_fs;
struct lfs_config *lfs_cfg;

#define FNAME_LEN 16
#define HEADER_LEN 64

#define SENSOR_BUF_LEN 256
#define GPS_BUF_LEN 256
#define STATE_BUF_LEN 256

static char s_filename[FNAME_LEN] = "dat_00.pb3";
static char s_gpsfname[FNAME_LEN] = "gps_00.pb3";
static char s_statefname[FNAME_LEN] = "fsl_00.pb3";
static char s_prffname[FNAME_LEN] = "prf_00.txt";

static struct lfs_file s_datfile;
static struct lfs_file s_gpsfile;
static struct lfs_file s_statefile;

// static pb_byte_t s_sensor_buffer[SENSOR_BUF_LEN];
// static pb_byte_t s_gps_buffer[GPS_BUF_LEN];
// static pb_byte_t s_state_buffer[STATE_BUF_LEN];

static const char s_header[HEADER_LEN] = FIRMWARE_SPECIFIER;

static Status nand_flash_create_sensor_file() {
    // Create sensor data file
    if (lfs_file_open(&s_fs, &s_datfile, s_filename,
                      LFS_O_CREAT | LFS_O_WRONLY) != LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write the header
    lfs_ssize_t bw = lfs_file_write(&s_fs, &s_datfile, s_header, HEADER_LEN);

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (lfs_file_sync(&s_fs, &s_datfile) != LFS_ERR_OK) {
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
    if (lfs_file_open(&s_fs, &s_gpsfile, s_gpsfname,
                      LFS_O_CREAT | LFS_O_WRONLY) != LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write the header
    lfs_ssize_t bw = lfs_file_write(&s_fs, &s_gpsfile, s_header, HEADER_LEN);

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (lfs_file_sync(&s_fs, &s_gpsfile) != LFS_ERR_OK) {
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
    if (lfs_file_open(&s_fs, &s_statefile, s_statefname,
                      LFS_O_CREAT | LFS_O_WRONLY) != LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write the header
    lfs_ssize_t bw = lfs_file_write(&s_fs, &s_statefile, s_header, HEADER_LEN);

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (lfs_file_sync(&s_fs, &s_statefile) != LFS_ERR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check that the frame will at least nominally fit in our buffer
    if (STATE_BUF_LEN < sizeof(StateFrame)) {
        return STATUS_DATA_ERROR;
    }

    return STATUS_OK;
}

// From user geky on github
int lfs_ls(lfs_t *lfs, const char *path) {
    lfs_dir_t dir;
    int err = lfs_dir_open(lfs, &dir, path);
    if (err) {
        return err;
    }

    struct lfs_info info;
    while (true) {
        int res = lfs_dir_read(lfs, &dir, &info);
        if (res < 0) {
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

        static const char *prefixes[] = {"", "K", "M", "G"};
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

Status nand_flash_init() {
    if (mt29f4g_init() != STATUS_OK) {
        printf("Failed to initialize NAND hardware\n");
        return STATUS_HARDWARE_ERROR;
    }

    lfs_cfg = mt29f4g_get_lfs_config();
    int status = lfs_mount(&s_fs, lfs_cfg);
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
#else
        return STATUS_HARDWARE_ERROR;
#endif
    }

    // See the files in the root directory
    if (lfs_ls(&s_fs, "/") != 0) {
        printf("Failed to list files on flash\n");
        return STATUS_HARDWARE_ERROR;
    }

    // See the space remaining
    lfs_ssize_t fs_size = lfs_fs_size(&s_fs);
    if (fs_size < 0) {
        printf("Failed to get file system space\n");
        return STATUS_HARDWARE_ERROR;
    } else {
        lfs_size_t fs_free = (lfs_cfg->block_count * lfs_cfg->block_size);
        static const char *prefixes[] = {"", "K", "M", "G"};
        for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
            if (fs_free >= (1 << 10 * i) - 1) {
                printf("Remaining space: %*lu%sB\n", 4 - (i != 0),
                       fs_free >> 10 * i, prefixes[i]);
                break;
            }
        }
    }

    // Increment the suffix of the filename until we find an unused name
    // I'll do this properly at some point I swear
    struct lfs_info info;
    while (lfs_stat(&s_fs, s_filename, &info) == LFS_ERR_OK) {
        if (s_filename[5] == '9') {
            if (s_filename[4] == '9') {
                return STATUS_DATA_ERROR;
            }
            s_filename[4] += 1;
            s_filename[5] = '0';
        } else {
            s_filename[5] += 1;
        }
    }
    s_gpsfname[4] = s_filename[4];
    s_gpsfname[5] = s_filename[5];
    s_statefname[4] = s_filename[4];
    s_statefname[5] = s_filename[5];
    s_prffname[4] = s_filename[4];
    s_prffname[5] = s_filename[5];

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