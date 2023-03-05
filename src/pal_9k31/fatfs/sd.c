#include "sd.h"

#include "ff.h"

static FATFS s_fs;
static char s_filename[12] = "data_00.csv";

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

    return STATUS_OK;
}
