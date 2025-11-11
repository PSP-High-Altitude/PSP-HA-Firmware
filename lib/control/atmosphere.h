#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

#define G_MAG \
    (9.81f)  // since including all of state_estimation.h would be a circular
             // dependency
#define R_MAG (287.05f)
#define TABLE_LEN (8)

// FOR PSPSP

typedef float float32_t;  // this is the size of float we will use

typedef struct {
    float altitude_table[TABLE_LEN];
    float temp_table[TABLE_LEN];
    float pressure_table[TABLE_LEN];
    float lapse_rate_table[TABLE_LEN];
} LayerData;

// FUNCTION DECLARATIONS HERE:
float calc_temp(float altitude, float initial_altitude, float initial_temp,
                float lapse_rate);
float calc_pressure(float altitude, float initial_altitude, float initial_temp,
                    float initial_pressure, float lapse_rate);
void gen_atmosphere_struct(LayerData* atmos, float initial_temp,
                           float initial_pressure);
float pressure_to_altitude(float pressure, LayerData* atmos);

#endif  // ATMOSPHERE_H
