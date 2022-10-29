#ifndef MS5637_H
#define MS5637_H

#include <math.h>
#include <stdint.h>

#include "data.h"
#include "i2c/i2c.h"

typedef enum {
    OSR_256 = 0x0,
    OSR_512 = 0x2,
    OSR_1024 = 0x4,
    OSR_2048 = 0x6,
    OSR_4096 = 0x8,
    OSR_8192 = 0xA,
} AdcSpeed;

typedef struct {
    uint16_t C1;
    uint16_t C2;
    uint16_t C3;
    uint16_t C4;
    uint16_t C5;
    uint16_t C6;
} CalibrationData;

#define D_READ_ERROR 0xFFFFFFFF

Status ms5637_init(I2cDevice* device);
BaroData ms5637_read(I2cDevice* device, AdcSpeed speed);

#endif // MS5637_H
