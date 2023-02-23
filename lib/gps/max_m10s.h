#ifndef MAX_M10S_H
#define MAX_M10S_H

#include "i2c/i2c.h"
#include "status.h"

typedef enum {
    MAX_M10S_LAYER_RAM = 0,
    MAX_M10S_LAYER_BBR = 1,
    MAX_M10S_LAYER_FLASH = 2,
    MAX_M10S_LAYER_DEFAULT = 7,
} Max_M10S_Layer_TypeDef;

Status max_m10s_init(I2cDevice* device);

#endif