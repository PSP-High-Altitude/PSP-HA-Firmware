#ifndef PRESSURE_ALTITUDE
#define PRESSURE_ALTITUDE

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define SEA_LEVEL_PRESSURE 1013.25  // Standard sea level pressure in millibars
#define LAPSE_RATE 0.0065           // Standard temperature lapse rate in K/m

float pressureToAltitude(float);

#endif  // PRESSURE_ALTITUDE
