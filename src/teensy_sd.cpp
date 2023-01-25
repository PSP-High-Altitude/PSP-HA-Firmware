#include "teensy_sd.h"

static char s_filename[12] = "data_00.csv";

Status sd_init(SDDevice* device) {
    if (!SD.begin(device->cs)) {
        return HARDWARE_ERROR;
    }

    // Increment the suffix of the filename until we find an unused name
    // This is a super shitty and inefficient way to do this, but this is also
    // supposed to be stopgap code that runs on a 600 MHz Cortex-M7, so eh
    while (SD.exists(s_filename)) {
        if (s_filename[6] == '9') {
            s_filename[5] += 1;
            s_filename[6] = '0';
        } else {
            s_filename[6] += 1;
        }
    }

    File file = SD.open(s_filename, FILE_WRITE);
    if (!file) {
        return ERROR;
    }

    file.println(
        "Timestamp,"
        "Ax,Ay,Az,"
        "Rx,Ry,Rz,"
        "Temp,"
        "Pressure");

    file.close();

    return OK;
}

Status sd_write(uint64_t timestamp, Accel* accel, Gyro* gyro, BaroData* baro) {
    File file = SD.open(s_filename, FILE_WRITE);

    int retval =
        file.printf("%d,%f,%f,%f,%f,%f,%f,%f,%f\n", timestamp, accel->accelX,
                    accel->accelY, accel->accelZ, gyro->gyroX, gyro->gyroY,
                    gyro->gyroZ, baro->temperature, baro->pressure);

    file.close();

    if (retval > 0) {
        return OK;
    } else {
        return ERROR;
    }
}
