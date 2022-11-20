#ifndef TEENSY_SD_H
#define TEENSY_SD_H

#include <SD.h>
#include <stdint.h>

#include "status.h"

typedef struct {
    uint8_t cs;
} SDDevice;

Status sd_init(SDDevice* device);

#endif  // TEENSY_SD_H
