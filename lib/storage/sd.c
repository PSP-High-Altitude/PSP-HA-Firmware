#include "sd.h"

#include "fatfs/ff.h"
#include "pb_encode.h"
#include "stdio.h"
#include "timer.h"

#define FNAME_LEN 16
#define HEADER_LEN 64

#define SENSOR_BUF_LEN 256
#define GPS_BUF_LEN 256
#define STATE_BUF_LEN 256

#define SD_MOUNT_POINT "/SD"

static FATFS s_fs;

static char s_filename[FNAME_LEN] = SD_MOUNT_POINT "/dat_00.pb3";
static char s_gpsfname[FNAME_LEN] = SD_MOUNT_POINT "/gps_00.pb3";
static char s_statefname[FNAME_LEN] = SD_MOUNT_POINT "/fsl_00.pb3";

static FIL s_datfile;
static FIL s_gpsfile;
static FIL s_statefile;

static pb_byte_t s_sensor_buffer[SENSOR_BUF_LEN];
static pb_byte_t s_gps_buffer[GPS_BUF_LEN];
static pb_byte_t s_state_buffer[STATE_BUF_LEN];

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
    if (SENSOR_BUF_LEN < sizeof(StateFrame)) {
        return STATUS_DATA_ERROR;
    }

    return STATUS_OK;
}

Status sd_init(SdDevice* dev) {
    diskio_init(dev);
    if (f_mount(&s_fs, SD_MOUNT_POINT, 0) != FR_OK) {
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

Status sd_write_sensor_data(SensorFrame* frame) {
    pb_ostream_t sensor_ostream =
        pb_ostream_from_buffer(s_sensor_buffer, SENSOR_BUF_LEN);

    bool encode_success = pb_encode_ex(&sensor_ostream, &SensorFrame_msg, frame,
                                       PB_ENCODE_DELIMITED);
    if (!encode_success) {
        return STATUS_ERROR;
    }

    UINT bw;
    if (f_write(&s_datfile, s_sensor_buffer, sensor_ostream.bytes_written,
                &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_write_gps_data(GpsFrame* frame) {
    pb_ostream_t gps_ostream =
        pb_ostream_from_buffer(s_gps_buffer, GPS_BUF_LEN);

    bool encode_success =
        pb_encode_ex(&gps_ostream, &GpsFrame_msg, frame, PB_ENCODE_DELIMITED);
    if (!encode_success) {
        return STATUS_ERROR;
    }

    UINT bw;
    if (f_write(&s_gpsfile, s_gps_buffer, gps_ostream.bytes_written, &bw) !=
        FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_write_state_data(StateFrame* frame) {
    pb_ostream_t state_ostream =
        pb_ostream_from_buffer(s_state_buffer, STATE_BUF_LEN);

    bool encode_success = pb_encode_ex(&state_ostream, &StateFrame_msg, frame,
                                       PB_ENCODE_DELIMITED);
    if (!encode_success) {
        return STATUS_ERROR;
    }

    UINT bw;
    if (f_write(&s_statefile, s_state_buffer, state_ostream.bytes_written,
                &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_reinit() {
    if (hal_reinit_card() != STATUS_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_mount(&s_fs, SD_MOUNT_POINT, 0) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_open(&s_datfile, s_filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (f_open(&s_gpsfile, s_gpsfname, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
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

    return STATUS_OK;
}
