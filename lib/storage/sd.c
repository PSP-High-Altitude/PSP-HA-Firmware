#include "sd.h"

#include "fatfs/ff.h"
#include "pb_create.h"
#include "stdio.h"
#include "timer.h"

#define FNAME_LEN 16
#define HEADER_LEN 64

#define SD_MOUNT_POINT "/SD"

static FATFS g_fs;

static char s_filename[FNAME_LEN] = SD_MOUNT_POINT "/dat_00.pb3";
static char s_gpsfname[FNAME_LEN] = SD_MOUNT_POINT "/gps_00.pb3";
static char s_statefname[FNAME_LEN] = SD_MOUNT_POINT "/fsl_00.pb3";
static char s_prffname[FNAME_LEN] = SD_MOUNT_POINT "/prf_00.txt";

static FIL s_datfile;
static FIL s_gpsfile;
static FIL s_statefile;

static const char s_header[HEADER_LEN] = FIRMWARE_SPECIFIER;

static Status sd_create_sensor_file() {
    // Create sensor data file
    if (f_open(&s_datfile, s_filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write the header
    UINT bw;
    if (f_write(&s_datfile, s_header, HEADER_LEN, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (f_sync(&s_datfile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check that the frame will at least nominally fit in our buffer
    if (SENSOR_BUF_LEN < sizeof(SensorFrame)) {
        return STATUS_DATA_ERROR;
    }

    return STATUS_OK;
}

static Status sd_create_gps_file() {
    // Create gps data file
    if (f_open(&s_gpsfile, s_gpsfname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write the header
    UINT bw;
    if (f_write(&s_gpsfile, s_header, HEADER_LEN, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
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

static Status sd_create_state_file() {
    // Create state data file
    if (f_open(&s_statefile, s_statefname, FA_CREATE_ALWAYS | FA_WRITE) !=
        FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write the header
    UINT bw;
    if (f_write(&s_statefile, s_header, HEADER_LEN, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check the correct number of bytes was written
    if (bw != HEADER_LEN) {
        return STATUS_HARDWARE_ERROR;
    }

    // Flush the header to disk
    if (f_sync(&s_statefile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check that the frame will at least nominally fit in our buffer
    if (STATE_BUF_LEN < sizeof(StateFrame)) {
        return STATUS_DATA_ERROR;
    }

    return STATUS_OK;
}

Status sd_init(SdDevice* dev) {
    diskio_init(dev);
    if (f_mount(&g_fs, SD_MOUNT_POINT, 1) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Increment the suffix of the filename until we find an unused name
    // I'll do this properly at some point I swear
    while (f_stat(s_filename, 0) == FR_OK) {
        if (s_filename[6 + strlen(SD_MOUNT_POINT)] == '9') {
            if (s_filename[5 + strlen(SD_MOUNT_POINT)] == '9') {
                return STATUS_DATA_ERROR;
            }
            s_filename[5 + strlen(SD_MOUNT_POINT)] += 1;
            s_filename[6 + strlen(SD_MOUNT_POINT)] = '0';
        } else {
            s_filename[6 + strlen(SD_MOUNT_POINT)] += 1;
        }
    }
    s_gpsfname[5 + strlen(SD_MOUNT_POINT)] =
        s_filename[5 + strlen(SD_MOUNT_POINT)];
    s_gpsfname[6 + strlen(SD_MOUNT_POINT)] =
        s_filename[6 + strlen(SD_MOUNT_POINT)];
    s_statefname[5 + strlen(SD_MOUNT_POINT)] =
        s_filename[5 + strlen(SD_MOUNT_POINT)];
    s_statefname[6 + strlen(SD_MOUNT_POINT)] =
        s_filename[6 + strlen(SD_MOUNT_POINT)];
    s_prffname[5 + strlen(SD_MOUNT_POINT)] =
        s_filename[5 + strlen(SD_MOUNT_POINT)];
    s_prffname[6 + strlen(SD_MOUNT_POINT)] =
        s_filename[6 + strlen(SD_MOUNT_POINT)];

    // Initialize the sensor file and stream
    Status sensor_status = sd_create_sensor_file();
    if (sensor_status != STATUS_OK) {
        return sensor_status;
    }

    // Initialize the GPS file and stream
    Status gps_status = sd_create_gps_file();
    if (gps_status != STATUS_OK) {
        return gps_status;
    }

    // Initialize the flight state file and stream
    Status state_status = sd_create_state_file();
    if (state_status != STATUS_OK) {
        return state_status;
    }

    return STATUS_OK;
}

Status sd_write_sensor_data(pb_byte_t* frame, size_t size) {
    if (frame == NULL) {
        return STATUS_ERROR;
    }

    UINT bw;
    if (f_write(&s_datfile, frame, size, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_write_gps_data(pb_byte_t* frame, size_t size) {
    if (frame == NULL) {
        return STATUS_ERROR;
    }

    UINT bw;
    if (f_write(&s_gpsfile, frame, size, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_write_state_data(pb_byte_t* frame, size_t size) {
    if (frame == NULL) {
        return STATUS_ERROR;
    }

    UINT bw;
    if (f_write(&s_statefile, frame, size, &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_reinit() {
    if (hal_reinit_card() != STATUS_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_mount(&g_fs, SD_MOUNT_POINT, 0) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_open(&s_datfile, s_filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_open(&s_gpsfile, s_gpsfname, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_open(&s_statefile, s_statefname, FA_OPEN_APPEND | FA_WRITE) !=
        FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_deinit() {
    if (f_close(&s_datfile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_close(&s_gpsfile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_close(&s_statefile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_unmount(SD_MOUNT_POINT) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_flush() {
    if (f_sync(&s_datfile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_sync(&s_gpsfile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_sync(&s_statefile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_dump_prf_stats(char stats[]) {
    FIL prffile;
    UINT bw = 0;

    // Create performance dump file
    if (f_open(&prffile, s_prffname, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write a timestamp
    bw += f_printf(&prffile, "%lu ms: ", (uint32_t)MILLIS());

    // Write the stats
    bw += f_printf(&prffile, stats);

    // Check that something was written
    if (bw == 0) {
        return STATUS_HARDWARE_ERROR;
    }

    // Close the file
    if (f_close(&prffile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}
