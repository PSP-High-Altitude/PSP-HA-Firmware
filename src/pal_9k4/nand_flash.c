#include "nand_flash.h"

#include "main.h"
#include "pb_create.h"
#include "stdio.h"
#include "timer.h"
#include "usbd_mtp_if.h"
#include "yaffs2/yaffs_driver.h"
#include "yaffs2/yaffsfs.h"

#define FNAME_LEN 16
#define FDIR_LEN 32
#define HEADER_LEN 64

extern uint8_t usb_mode;
extern int yaffs_err;

static char s_fltdata_dir[FDIR_LEN] = "/";
static char s_filename[FNAME_LEN + FDIR_LEN] = "dat_00.pb3";
static char s_gpsfname[FNAME_LEN + FDIR_LEN] = "gps_00.pb3";
static char s_statefname[FNAME_LEN + FDIR_LEN] = "fsl_00.pb3";
static char s_prffname[FNAME_LEN + FDIR_LEN] = "prf_00.txt";

static int s_datfile;
static int s_gpsfile;
static int s_statefile;
static uint8_t s_datfile_open = 0;
static uint8_t s_gpsfile_open = 0;
static uint8_t s_statefile_open = 0;

struct yaffs_dev dev;

// static pb_byte_t s_sensor_buffer[SENSOR_BUF_LEN];
// static pb_byte_t s_gps_buffer[GPS_BUF_LEN];
// static pb_byte_t s_state_buffer[STATE_BUF_LEN];

static const char s_header[HEADER_LEN] = FIRMWARE_SPECIFIER;

extern uint32_t mtp_file_idx;

static Status nand_flash_create_sensor_file() {
    // Create sensor data file
    if ((s_datfile = yaffs_open(s_filename, O_CREAT | O_WRONLY,
                                S_IREAD | S_IWRITE)) < 0) {
        return STATUS_HARDWARE_ERROR;
    }
    s_datfile_open = 1;

    // Write the header
    int bw = yaffs_write(s_datfile, s_header, HEADER_LEN);

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (yaffs_sync(s_filename) != 0) {
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
    if ((s_gpsfile = yaffs_open(s_gpsfname, O_CREAT | O_WRONLY,
                                S_IREAD | S_IWRITE)) < 0) {
        return STATUS_HARDWARE_ERROR;
    }
    s_gpsfile_open = 1;

    // Write the header
    int bw = yaffs_write(s_gpsfile, s_header, HEADER_LEN);

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (yaffs_sync(s_gpsfname) != 0) {
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
    if ((s_statefile = yaffs_open(s_statefname, O_CREAT | O_WRONLY,
                                  S_IREAD | S_IWRITE)) < 0) {
        return STATUS_HARDWARE_ERROR;
    }
    s_statefile_open = 1;

    // Write the header
    int bw = yaffs_write(s_statefile, s_header, HEADER_LEN);

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (yaffs_sync(s_statefname) != 0) {
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
    yaffs_DIR* dir = yaffs_opendir(s_fltdata_dir);
    if (dir == NULL) {
        return STATUS_ERROR;
    }

    int32_t min_file_suffix = 100;
    int32_t max_file_suffix = -1;

    struct yaffs_dirent* info;
    while (true) {
        info = yaffs_readdir(dir);
        if (info == NULL && yaffs_err != 0) {
            yaffs_closedir(dir);
            return STATUS_ERROR;
        }

        if (info == NULL) {
            break;
        }

        struct yaffs_stat stat;
        char str[128];
        if (snprintf(str, 128, "%s%s", s_fltdata_dir, info->d_name) < 0) {
            yaffs_closedir(dir);
            return STATUS_ERROR;
        }
        if (yaffs_lstat(str, &stat) != 0) {
            yaffs_closedir(dir);
            return STATUS_ERROR;
        }

        if ((stat.st_mode & S_IFMT) != S_IFREG) {
            continue;
        }

        if (strncmp(info->d_name, "dat_", 4) == 0) {
            int32_t suffix = atoi(info->d_name + 4);
            if (suffix > max_file_suffix) {
                max_file_suffix = suffix;
            }
            if (suffix < min_file_suffix) {
                min_file_suffix = suffix;
            }
        }
    }

    yaffs_closedir(dir);

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
        yaffs_unlink(s_filename);
        yaffs_unlink(s_gpsfname);
        yaffs_unlink(s_statefname);
        yaffs_unlink(s_prffname);
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

Status nand_flash_write_sensor_data(pb_byte_t* frame, size_t size) {
    if (frame == NULL) {
        return STATUS_ERROR;
    }

    if (!s_datfile_open) {
        return STATUS_ERROR;
    }

    if (yaffs_write(s_datfile, (uint8_t*)frame, size) != 0) {
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

    if (yaffs_write(s_gpsfile, (uint8_t*)frame, size) != 0) {
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

    if (yaffs_write(s_statefile, (uint8_t*)frame, size) != 0) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_dump_prf_stats(char stats[]) {
    int prffile;
    size_t bw = 0;

    // Create performance dump file
    if ((prffile = yaffs_open(s_prffname, O_APPEND | O_CREAT | O_WRONLY,
                              S_IREAD | S_IWRITE)) < 0) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write a timestamp
    char timestamp[32];
    int size = snprintf(timestamp, 32, "%lu ms:\n", (uint32_t)MILLIS());
    if (size > 0) {
        bw += yaffs_write(prffile, timestamp, size);
    }

    // Write the stats
    size = strlen(stats);
    bw += yaffs_write(prffile, stats, size);

    // Check that something was written
    if (bw == 0) {
        return STATUS_HARDWARE_ERROR;
    }

    // Close the file
    if (yaffs_close(prffile) != 0) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status nand_flash_init() {
    if (mt29f4g_init() != STATUS_OK) {
        printf("Failed to initialize NAND hardware\n");
        return STATUS_HARDWARE_ERROR;
    }

    yaffsfs_OSInitialisation();
    yaffs_driver_init(&dev);

    int status = yaffs_mount("/");
#ifdef NAND_ALLOW_REFORMAT
    if (status != 0) {
        printf("No file system found. Formatting...\n");
        // Fully erase the chip before formatting
        status = mt29f4g_erase_chip();
        if (status != STATUS_OK) {
            printf("Failed to erase NAND: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
        status = yaffs_format("/", 0, 0, 0);
        if (status != 0) {
            printf("Failed to format NAND: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
        status = yaffs_mount("/");
        if (status != 0) {
            printf("Failed to mount NAND: %d\n", status);
            return STATUS_HARDWARE_ERROR;
        }
    }
#else
    if (status != 0) {
        printf("Failed to mount NAND: %d\n", status);
        return STATUS_HARDWARE_ERROR;
    }
#endif
    printf("NAND mounted successfully!\n");

    if (usb_mode == 1) {
        // Open the files
        Status open_file_stat = nand_flash_open_files();
        if (open_file_stat != STATUS_OK) {
            return open_file_stat;
        }
    }

    // See the files in the root directory
    if (yaffs_ls("/") != 0) {
        printf("Failed to list files on flash\n");
        return STATUS_HARDWARE_ERROR;
    }

    // See the space remaining
    // lfs_ssize_t fs_size = lfs_fs_size(&g_fs);
    // if (fs_size < 0) {
    //    printf("Failed to get file system space\n");
    //    return STATUS_HARDWARE_ERROR;
    //} else {
    //    lfs_size_t fs_free =
    //        (g_lfs_cfg->block_count * g_lfs_cfg->block_size) - fs_size;
    //    static const char *prefixes[] = {"", "K", "M", "G"};
    //    for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--)
    //    {
    //        if (fs_free >= (1 << 10 * i) - 1) {
    //            printf("Remaining space: %*lu%sB\n", 4 - (i != 0),
    //                   fs_free >> 10 * i, prefixes[i]);
    //            break;
    //        }
    //    }
    //}

    return STATUS_OK;
}

Status nand_flash_reinit() {
    if (!s_datfile_open) {
        if ((s_datfile = yaffs_open(s_filename, O_APPEND | O_WRONLY,
                                    S_IWRITE | S_IREAD)) < 0) {
            return STATUS_HARDWARE_ERROR;
            s_datfile_open = 0;
        }
        s_datfile_open = 1;
    }
    if (!s_gpsfile_open) {
        if ((s_gpsfile = yaffs_open(s_gpsfname, O_APPEND | O_WRONLY,
                                    S_IWRITE | S_IREAD)) < 0) {
            return STATUS_HARDWARE_ERROR;
            s_gpsfile_open = 0;
        }
        s_gpsfile_open = 1;
    }
    if (!s_statefile_open) {
        if ((s_statefile = yaffs_open(s_statefname, O_APPEND | O_WRONLY,
                                      S_IWRITE | S_IREAD)) < 0) {
            return STATUS_HARDWARE_ERROR;
            s_statefile_open = 0;
        }
        s_statefile_open = 1;
    }

    return STATUS_OK;
}

Status nand_flash_deinit() {
    if (s_datfile_open) {
        if (yaffs_close(s_datfile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
        s_datfile_open = 0;
    }
    if (s_gpsfile_open) {
        if (yaffs_close(s_gpsfile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
        s_gpsfile_open = 0;
    }
    if (s_statefile_open) {
        if (yaffs_close(s_statefile) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
        s_statefile_open = 0;
    }

    return STATUS_OK;
}

Status nand_flash_flush() {
    if (s_datfile_open) {
        if (yaffs_sync(s_filename) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
    }
    if (s_gpsfile_open) {
        if (yaffs_sync(s_gpsfname) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
    }
    if (s_statefile_open) {
        if (yaffs_sync(s_statefname) != 0) {
            return STATUS_HARDWARE_ERROR;
        }
    }

    return STATUS_OK;
}

int yaffs_ls(const char* path) {
    yaffs_DIR* dir = yaffs_opendir(path);
    if (dir == NULL) {
        return STATUS_ERROR;
    }

    struct yaffs_dirent* info;
    while (true) {
        info = yaffs_readdir(dir);
        if (info == NULL && yaffs_err != 0) {
            yaffs_closedir(dir);
            return STATUS_ERROR;
        }

        if (info == NULL) {
            break;
        }

        struct yaffs_stat stat;
        char str[128];
        if (snprintf(str, 128, "%s%s", path, info->d_name) < 0) {
            yaffs_closedir(dir);
            return STATUS_ERROR;
        }
        if (yaffs_lstat(str, &stat) != 0) {
            yaffs_closedir(dir);
            return STATUS_ERROR;
        }

        switch (stat.st_mode & S_IFMT) {
            case S_IFREG:
                printf("reg ");
                break;
            case S_IFDIR:
                printf("dir ");
                break;
            default:
                printf("?   ");
                break;
        }

        static const char* prefixes[] = {"", "K", "M", "G"};
        for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
            if (stat.st_size >= (1 << 10 * i) - 1) {
                printf("%*lu%sB ", 4 - (i != 0), stat.st_size >> 10 * i,
                       prefixes[i]);
                break;
            }
        }

        printf("%s\n", info->d_name);
    }

    if (yaffs_closedir(dir) != 0) {
        return STATUS_ERROR;
    }

    return 0;
}