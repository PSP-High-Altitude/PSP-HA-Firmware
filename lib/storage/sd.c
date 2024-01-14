#include "sd.h"

#include "fatfs/ff.h"
#include "pb_encode.h"
#include "timer.h"

#define FNAME_LEN 16
#define HEADER_LEN 64

#define SENSOR_BUF_LEN 256

static FATFS s_fs;

static char s_filename[FNAME_LEN] = "dat_00.csv";
static char s_gpsfname[FNAME_LEN] = "gps_00.csv";

static FIL s_datfile;
static FIL s_gpsfile;

static pb_ostream_t s_sensor_ostream;
static pb_byte_t s_sensor_buffer[SENSOR_BUF_LEN];

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
    if (f_sync(&s_gpsfile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check that the frame will at least nominally fit in our buffer
    if (SENSOR_BUF_LEN < sizeof(SensorFrame)) {
        return STATUS_DATA_ERROR;
    }

    // Create the nanopb output file stream
    s_sensor_ostream = pb_ostream_from_buffer(s_sensor_buffer, SENSOR_BUF_LEN);

    return STATUS_OK;
}

Status sd_init(SdDevice* dev) {
    diskio_init(dev);

    if (f_mount(&s_fs, "", 0) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Increment the suffix of the filename until we find an unused name
    // I'll do this properly at some point I swear
    while (f_stat(s_filename, 0) == FR_OK) {
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

    // Initialize the sensor file and stream
    Status sensor_status = sd_create_sensor_file();
    if (sensor_status != STATUS_OK) {
        return sensor_status;
    }

    return STATUS_OK;
}

Status sd_write_sensor_data(SensorFrame* frame) {
    s_sensor_ostream.bytes_written = 0;

    bool encode_success = pb_encode_ex(&s_sensor_ostream, &SensorFrame_msg,
                                       frame, PB_ENCODE_DELIMITED);
    if (encode_success) {
        return STATUS_OK;
    } else {
        return STATUS_ERROR;
    }
}

Status sd_reinit() {
    if (f_mount(&s_fs, "", 0) != FR_OK) {
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
    if (f_unmount("") != FR_OK) {
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