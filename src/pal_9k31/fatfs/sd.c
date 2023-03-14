#include "sd.h"

#include "ff.h"
#include "timer.h"

static FATFS s_fs;
static char s_filename[12] = "data_00.csv";

DWORD get_fattime() { return 0; }

Status sd_init(SpiDevice* dev) {
    sd_spi_init(dev);

    if (f_mount(&s_fs, "", 0) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    // Increment the suffix of the filename until we find an unused name
    // I'll do this properly at some point I swear
    while (f_stat(s_filename, 0) == FR_OK) {
        if (s_filename[6] == '9') {
            if (s_filename[5] == '9') {
                return STATUS_DATA_ERROR;
            }
            s_filename[5] += 1;
            s_filename[6] = '0';
        } else {
            s_filename[6] += 1;
        }
    }

    FIL file;
    if (f_open(&file, s_filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    f_printf(
        &file,
        "Timestamp,Ax,Ay,Az,Rx,Ry,Rz,Temp,Pressure,Mx,My,Mz,Lat,Lon,Alt\n");
    if (f_close(&file) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}

Status sd_write(uint64_t timestamp, Accel* accel, Gyro* gyro, BaroData* baro,
                Mag* mag, GPS_Fix_TypeDef* fix) {
    FIL file;
    if (f_open(&file, s_filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    f_printf(&file, "%lld,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,", timestamp,
             accel->accelX, accel->accelY, accel->accelZ, gyro->gyroX,
             gyro->gyroY, gyro->gyroZ);
    f_printf(&file, "%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f\n",
             baro->temperature, baro->pressure, mag->magX, mag->magY, mag->magZ,
             fix->lat, fix->lon, fix->height_msl);
    if (f_close(&file) != FR_OK) {
        return STATUS_HARDWARE_ERROR;
    }

    return STATUS_OK;
}
