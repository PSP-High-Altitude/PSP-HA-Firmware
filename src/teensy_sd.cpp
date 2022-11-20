#include "teensy_sd.h"

static File s_file;

Status sd_init(SDDevice* device) {
    if (!SD.begin(device->cs)) {
        return HARDWARE_ERROR;
    }

    char filename[8] = "data_00";

    // Increment the suffix of the filename until we find an unused name
    // This is a super shitty and inefficient way to do this, but this is also
    // supposed to be stopgap code that runs on a 600 MHz Cortex-M7, so eh
    while (SD.exists(filename)) {
        if (filename[6] == '9') {
            filename[5] += 1;
            filename[6] = '0';
        } else {
            filename[6] += 1;
        }
    }

    s_file = SD.open(filename, FILE_WRITE);
    if (!s_file) {
        return ERROR;
    }

    return OK;
}
