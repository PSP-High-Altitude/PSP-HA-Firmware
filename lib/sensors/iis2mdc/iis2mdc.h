#ifndef IIS2MDC_H
#define IIS2MDC_H

#include <stdint.h>

#include "data.h"
#include "i2c/i2c.h"
#include "status.h"

// Register Definitions
#define IIS2MDC_OFFSET 0x45  // Start of offset registers (3x2 bytes)
#define IIS2MDC_WHO_AM_I 0x4f
#define IIS2MDC_CFG_A 0x60
#define IIS2MDC_CFG_B 0x61
#define IIS2MDC_CFG_C 0x62
#define IIS2MDC_OUT 0x68       // Start of output registers (3x2 bytes)
#define IIS2MDC_TEMP_OUT 0x6e  // Temp register (1x2 bytes)

// Settings
typedef enum {
    IIS2MDC_ODR_10_HZ = 0,
    IIS2MDC_ODR_20_HZ = 1,
    IIS2MDC_ODR_50_HZ = 2,
    IIS2MDC_ODR_100_HZ = 3,
} Iis2mdcODR;

// Initalize the sensor
Status iis2mdc_init(I2cDevice* device, Iis2mdcODR odr);

// Read the acceleration registers
Mag iis2mdc_read(I2cDevice* device);

#endif  // IIS2MDC_H
