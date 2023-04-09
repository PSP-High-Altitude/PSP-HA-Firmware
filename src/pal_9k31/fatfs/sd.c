#include "sd.h"

#include "ff.h"
#include "timer.h"

static FATFS s_fs;
static char s_filename[12] = "dat_00.csv";
static char s_gpsfname[12] = "gps_00.csv";
static FIL s_datfile;
static FIL s_gpsfile;

DWORD get_fattime() { return 0; }

Status sd_init(SpiDevice* dev) {
    sd_spi_init(dev);

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

    // Create sensor data file and header
    if (f_open(&s_datfile, s_filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    f_printf(&s_datfile,
             "Timestamp,Ax,Ay,Az,Rx,Ry,Rz,Temp,Pressure,Mx,My,Mz,Gx,Gy,Gz\n");
    if (f_sync(&s_datfile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Create GPS file and header
    if (f_open(&s_gpsfile, s_gpsfname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    f_printf(
        &s_gpsfile,
        "Timestamp,Datetime,Lon,Lat,Alt,AccH,AccV,VelN,VelE,VelD,Invalid\n");
    if (f_sync(&s_gpsfile) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_write_sensor_data(volatile SensorData* data) {
    if (f_printf(&s_datfile,
                 "%llu,"                 // Timestamp
                 "%9.6f,%9.6f,%9.6f,"    // IMU acceleration
                 "%9.6f,%9.6f,%9.6f,"    // IMU rotation
                 "%9.6f,%9.6f,"          // Barometer
                 "%9.6f,%9.6f,%9.6f,"    // Magnetometer
                 "%9.6f,%9.6f,%9.6f\n",  // Accelerometer
                 //
                 data->timestamp,                                             //
                 data->accel.accelX, data->accel.accelY, data->accel.accelZ,  //
                 data->gyro.gyroX, data->gyro.gyroY, data->gyro.gyroZ,        //
                 data->baro.temperature, data->baro.pressure,                 //
                 data->mag.magX, data->mag.magY, data->mag.magZ,              //
                 data->acch.accelX, data->acch.accelY, data->acch.accelZ      //
                 ) <= 0) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_write_gps_data(uint64_t timestamp, volatile GPS_Fix_TypeDef* fix) {
    if (f_printf(&s_gpsfile,
                 "%llu,"                // Timestamp
                 "%u-%u-%uT%u:%u:%uZ,"  // Datetime
                 "%9.6f,%9.6f,%9.6f,"   // Lat, Lon, Alt
                 "%9.6f,%9.6f,"         // Acc
                 "%9.6f,%9.6f,%9.6f,"   // Vel
                 "%u\n",                // Invalid
                 //
                 timestamp,                                     // Timestamp
                 fix->year, fix->month, fix->day,               // Date
                 fix->hour, fix->min, fix->sec,                 // Time
                 fix->lat, fix->lon, fix->height_msl,           // Lat, Lon, Alt
                 fix->accuracy_horiz, fix->accuracy_vertical,   // Acc
                 fix->vel_north, fix->vel_east, fix->vel_down,  // Vel
                 fix->invalid_llh                               // Invalid
                 ) <= 0) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
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
