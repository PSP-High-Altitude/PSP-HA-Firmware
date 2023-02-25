#ifndef MAX_M10S_H
#define MAX_M10S_H

#include "i2c/i2c.h"
#include "status.h"

#define

typedef enum {
    MAX_M10S_LAYER_GET_RAM = 0,
    MAX_M10S_LAYER_GET_BBR = 1,
    MAX_M10S_LAYER_GET_FLASH = 2,
    MAX_M10S_LAYER_GET_DEFAULT = 7,
    MAX_M10S_LAYER_SET_RAM = 1,
    MAX_M10S_LAYER_SET_BBR = 2,
    MAX_M10S_LAYER_SET_FLASH = 4,
    MAX_M10S_LAYER_SET_ALL = 7,
} Max_M10S_Layer_TypeDef;

Status max_m10s_init(I2cDevice* device);

#endif