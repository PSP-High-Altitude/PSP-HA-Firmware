#include "atmosphere.h"  // include other files in quotes

#include <math.h>  // include external libraries in brackets
#include <stdlib.h>

// FOR PSPSP

// FUNCTION DEFINITIONS HERE

float atmos_calc_temp(float altitude, float initial_altitude,
                      float initial_temp, float lapse_rate) {
    return initial_temp + lapse_rate * (altitude - initial_altitude);
}

float atmos_calc_pressure(float altitude, float initial_altitude,
                          float initial_temp, float initial_pressure,
                          float lapse_rate) {
    float temp =
        atmos_calc_temp(altitude, initial_altitude, initial_temp, lapse_rate);

    if (lapse_rate == 0)
        return initial_pressure *
               expf(-1 * G_MAG * (altitude - initial_altitude) /
                    (R_MAG * temp));

    return initial_pressure *
           powf((temp / initial_temp), -1 * G_MAG / (R_MAG * lapse_rate));
}

void atmos_gen_atmosphere_struct(LayerData* atmos, float initial_temp,
                                 float initial_pressure) {
    atmos->temp_table[0] = initial_temp;
    atmos->pressure_table[0] = initial_pressure;

    /*
    for (int i = 0; i < TABLE_LEN; i++)
    {
            atmos->altitude_table[i] = altitude_table[i];
            atmos->lapse_rate_table[i] = lapse_rate_table[i];
    }
    */

    for (int i = 1; i < TABLE_LEN; i++) {
        atmos->temp_table[i] = atmos_calc_temp(
            atmos->altitude_table[i], atmos->altitude_table[i - 1],
            atmos->temp_table[i - 1], atmos->lapse_rate_table[i - 1]);
        atmos->pressure_table[i] = atmos_calc_pressure(
            atmos->altitude_table[i], atmos->altitude_table[i - 1],
            atmos->temp_table[i - 1], atmos->pressure_table[i - 1],
            atmos->lapse_rate_table[i - 1]);
    }
}

// TODO: optimize using rocket state instead of looping every time
// The worst case time complexity right now is O(n), but this could potentially
// be brought down to O(1)
float atmos_pressure_to_altitude(float pressure, LayerData* atmos) {
    int closest = -1;

    for (int i = 0; i < TABLE_LEN - 1;
         i++)  // since pressure decreases monotonically this is how we
               // determine altitude
    {
        if (pressure <= atmos->pressure_table[i] &&
            pressure > atmos->pressure_table[i + 1]) {
            closest = i;
            break;
        }
    }

    if (closest == -1)  // edge case (invalid pressure)
        return -1.0f;

    if (atmos->lapse_rate_table[closest] == 0.0f)
        return -1 * R_MAG * atmos->temp_table[closest] / G_MAG *
                   logf(pressure / atmos->pressure_table[closest]) +
               atmos->altitude_table[closest];

    return atmos->temp_table[closest] / atmos->lapse_rate_table[closest] *
               (powf(pressure / atmos->pressure_table[closest],
                     -1 * R_MAG * atmos->lapse_rate_table[closest] / G_MAG) -
                1) +
           atmos->altitude_table[closest];
}
