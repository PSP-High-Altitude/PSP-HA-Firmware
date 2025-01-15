#include "sd.h"

#include "fatfs/ff.h"
#include "stdio.h"
#include "timer.h"

#define FNAME_LEN 16

#define SENSOR_BUF_LEN 256
#define GPS_BUF_LEN 256
#define STATE_BUF_LEN 256

#define CSV_BUFFER_SIZE 512

#define SD_MOUNT_POINT "/SD"

static FATFS s_fs;

static char s_datfname[FNAME_LEN] = SD_MOUNT_POINT "/dat_00.csv";
static char s_prffname[FNAME_LEN] = SD_MOUNT_POINT "/prf_00.txt";

static FIL s_datfile;

static const char s_header[] =
    "sensor_timestamp,temperature,pressure,mag_x,mag_y,mag_z,"
    "gps_timestamp,utc_time,num_sats,lon,lat,height,height_msl,"
    "accuracy_horiz,accuracy_vertical,vel_north,vel_east,vel_down,"
    "ground_speed,hdg\n";

static Status sd_create_data_file() {
    // Create sensor data file
    if (f_open(&s_datfile, s_datfname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Write the header
    UINT bw;
    if (f_write(&s_datfile, s_header, sizeof(s_header), &bw) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Check the correct number of bytes was written
    if (bw != sizeof(s_header)) {
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

Status sd_init(SdDevice* dev) {
    diskio_init(dev);
    if (f_mount(&s_fs, SD_MOUNT_POINT, 0) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Increment the suffix of the filename until we find an unused name
    // I'll do this properly at some point I swear
    while (f_stat(s_datfname, 0) == FR_OK) {
        if (s_datfname[6 + strlen(SD_MOUNT_POINT)] == '9') {
            if (s_datfname[5 + strlen(SD_MOUNT_POINT)] == '9') {
                return STATUS_DATA_ERROR;
            }
            s_datfname[5 + strlen(SD_MOUNT_POINT)] += 1;
            s_datfname[6 + strlen(SD_MOUNT_POINT)] = '0';
        } else {
            s_datfname[6 + strlen(SD_MOUNT_POINT)] += 1;
        }
    }
    s_prffname[5 + strlen(SD_MOUNT_POINT)] =
        s_datfname[5 + strlen(SD_MOUNT_POINT)];
    s_prffname[6 + strlen(SD_MOUNT_POINT)] =
        s_datfname[6 + strlen(SD_MOUNT_POINT)];

    // Initialize the sensor file and stream
    Status sensor_status = sd_create_data_file();
    if (sensor_status != STATUS_OK) {
        return sensor_status;
    }

    return STATUS_OK;
}

Status sd_write_data(SensorFrame* sensor_frame, GpsFrame* gps_frame) {
    if (!sensor_frame || !gps_frame) {
        return STATUS_ERROR;
    }

    char buffer[CSV_BUFFER_SIZE];
    int len = snprintf(
        buffer, CSV_BUFFER_SIZE,
        // SensorFrame fields
        "%lu,"             // sensor_frame->timestamp
        "%.2f,"            // sensor_frame->temperature
        "%.2f,"            // sensor_frame->pressure
        "%.2f,%.2f,%.2f,"  // sensor_frame->mag_i_x, sensor_frame->mag_i_y,
                           // sensor_frame->mag_i_z

        // GpsFrame fields
        "%lu,"                                  // gps_frame->timestamp
        "%04lu-%02lu-%02luT%02lu:%02lu:%02lu,"  // gps_frame time
        "%lu,"                                  // gps_frame->num_sats
        "%.6f,"                                 // gps_frame->lon
        "%.6f,"                                 // gps_frame->lat
        "%.2f,"                                 // gps_frame->height
        "%.2f,"                                 // gps_frame->height_msl
        "%.2f,"                                 // gps_frame->accuracy_horiz
        "%.2f,"                                 // gps_frame->accuracy_vertical
        "%.2f,"                                 // gps_frame->vel_north
        "%.2f,"                                 // gps_frame->vel_east
        "%.2f,"                                 // gps_frame->vel_down
        "%.2f,"                                 // gps_frame->ground_speed
        "%.2f\n",                               // gps_frame->hdg

        // SensorFrame values
        (uint32_t)sensor_frame->timestamp, sensor_frame->temperature,
        sensor_frame->pressure, sensor_frame->mag_i_x, sensor_frame->mag_i_y,
        sensor_frame->mag_i_z,

        // GpsFrame values
        (uint32_t)gps_frame->timestamp, gps_frame->year, gps_frame->month, gps_frame->day,
        gps_frame->hour, gps_frame->min, gps_frame->sec, gps_frame->num_sats,
        gps_frame->lon, gps_frame->lat, gps_frame->height,
        gps_frame->height_msl, gps_frame->accuracy_horiz,
        gps_frame->accuracy_vertical, gps_frame->vel_north, gps_frame->vel_east,
        gps_frame->vel_down, gps_frame->ground_speed, gps_frame->hdg);

    if (len < 0 || len >= CSV_BUFFER_SIZE) {
        return STATUS_ERROR;  // Error in formatting or buffer overflow
    }

    UINT bytes_written;
    FRESULT result = f_write(&s_datfile, buffer, len, &bytes_written);

    if (result != FR_OK || bytes_written != (UINT)len) {
        return STATUS_ERROR;
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
    if (f_open(&s_datfile, s_datfname, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_deinit() {
    if (f_close(&s_datfile) != FR_OK) {
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
