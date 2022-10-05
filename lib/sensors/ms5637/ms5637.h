#ifndef MS5637_H
#define MS5637_H

#include <stdint.h>

typedef struct {
    uint16_t sens;     // Pressure Sensitivity
    uint16_t off;      // pressure offset
    uint16_t tcs;      // Temperature coefficient of pressure sensitivity
    uint16_t tco;      // Temperature coefficient of pressure offset
    uint16_t tref;     // Reference Temperature
    uint16_t tempsens; // Temperature coefficient of the temperature
} calibration; // This struct is used to store the calibrations values used in
               // order to calculate the pressure and temperature in usable
               // units (mbar and C)

#endif // MS5637_H
