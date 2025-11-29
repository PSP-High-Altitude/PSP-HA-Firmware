#include "atmosphere.h"  // include other files in quotes

#include <math.h>  // include external libraries in brackets
#include <stdlib.h>

// Static things
static LayerData layer_data = {
    .altitude_table = {0.f, 11000.f, 25200.f, 47000.f, 53000.f, 79000.f,
                       90000.f, 105000.f},  // m
    .lapse_rate_table = {-6.5e-3f, 0.0f, 3.0e-3f, 0.0f, -4.5e-3f, 0.0f, 4.0e-3f,
                         0.0f}};
// static float initial_altitude;
// static float initial_temp;
// static float initial_pressure;

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

    if (lapse_rate == 0)  // pause
        return initial_pressure *
               expf(-1 * G_MAG * (altitude - initial_altitude) /
                    (R_MAG * temp));

    // or sphere
    return initial_pressure *
           powf((temp / initial_temp), -1 * G_MAG / (R_MAG * lapse_rate));
}

float atmos_calc_pressure_deriv(float altitude) {
    // find layer
    int closest = -1;

    for (int i = 0; i < TABLE_LEN - 1; i++) {
        if (altitude >= layer_data.altitude_table[i] &&
            altitude < layer_data.altitude_table[i + 1]) {
            closest = i;
            break;
        }
    }
    if (closest == -1) return -1.0f;
    float initial_altitude = layer_data.altitude_table[closest];
    float initial_temp = layer_data.temp_table[closest];
    float initial_pressure = layer_data.pressure_table[closest];
    float lapse_rate = layer_data.lapse_rate_table[closest];

    float temp = atmos_calc_temp(altitude, initial_altitude, initial_temp,
                                 lapse_rate);

    if (lapse_rate == 0)  // pause
        return -1 * G_MAG / (R_MAG * temp) *
               atmos_calc_pressure(
                   altitude, initial_altitude, initial_temp, initial_pressure,
                   lapse_rate);  // since it is an exponential function, the
                                 // derivative is a multiple of itself
    // or sphere
    return -1 * G_MAG / (R_MAG * initial_temp) * initial_pressure *
           powf(temp / initial_temp, -1 * G_MAG / (R_MAG * lapse_rate) - 1);
}

void atmos_gen_atmosphere_struct(float initial_altitude, float initial_temp,
                                 float initial_pressure) {
    layer_data.altitude_table[0] = initial_altitude;
    layer_data.temp_table[0] = initial_temp;
    layer_data.pressure_table[0] = initial_pressure;

    for (int i = 1; i < TABLE_LEN; i++) {
        layer_data.temp_table[i] = atmos_calc_temp(
            layer_data.altitude_table[i], layer_data.altitude_table[i - 1],
            layer_data.temp_table[i - 1], layer_data.lapse_rate_table[i - 1]);
        layer_data.pressure_table[i] = atmos_calc_pressure(
            layer_data.altitude_table[i], layer_data.altitude_table[i - 1],
            layer_data.temp_table[i - 1], layer_data.pressure_table[i - 1],
            layer_data.lapse_rate_table[i - 1]);
    }
}

// TODO: optimize using rocket state instead of looping every time
// The worst case time complexity right now is O(n), but this could potentially
// be brought down to O(1)
float atmos_pressure_to_altitude(float pressure) {
    int closest = -1;

    for (int i = 0; i < TABLE_LEN - 1;
         i++)  // since pressure decreases monotonically this is how we
               // determine altitude
    {
        if (pressure <= layer_data.pressure_table[i] &&
            pressure > layer_data.pressure_table[i + 1]) {
            closest = i;
            break;
        }
    }

    if (closest == -1)  // edge case (invalid pressure)
        return -1.0f;

    if (layer_data.lapse_rate_table[closest] == 0.0f)
        return -1 * R_MAG * layer_data.temp_table[closest] / G_MAG *
                   logf(pressure / layer_data.pressure_table[closest]) +
               layer_data.altitude_table[closest];

    return layer_data.temp_table[closest] /
               layer_data.lapse_rate_table[closest] *
               (powf(
                    pressure / layer_data.pressure_table[closest],
                    -1 * R_MAG * layer_data.lapse_rate_table[closest] / G_MAG) -
                1) +
           layer_data.altitude_table[closest];
}

float atmos_altitude_to_pressure(float altitude) {
    int closest = -1;

    for (int i = 0; i < TABLE_LEN - 1; i++) {
        if (altitude >= layer_data.altitude_table[i] &&
            altitude < layer_data.altitude_table[i + 1]) {
            closest = i;
            break;
        }
    }

    if (closest == -1) return -1.0f;

    return atmos_calc_pressure(altitude, layer_data.altitude_table[closest],
                               layer_data.temp_table[closest],
                               layer_data.pressure_table[closest],
                               layer_data.lapse_rate_table[closest]);
}

void atmos_set_ground_alt(float ground_altitude) {
    layer_data.altitude_table[0] = ground_altitude;
    return;
}