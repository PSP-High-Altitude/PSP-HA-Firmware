#ifndef FLIGHT_ESTIMATION_H
#define FLIGHT_ESTIMATION_H

#include <stdio.h>

#include "data.h"
#include "vector.h"

// set these befire flight
#define MAIN_HEIGHT 300  // m
#define INIT_TIME 10     // s
#define ACC_BOOST 50     // m/s^2
#define ACC_COAST 0      // m/s^2
#define VEL_FAST 200     // m/s
#define IMU_UP -2        // +/- 1, 2, 3 for x, y, z
#define HIGH_G_UP 2      // +/- 1, 2, 3 for x, y, z
#define LOW_G_CUTOFF 14  // g
#define G 9.81           // m/s^2
#define DROGUE_V 0       // m/s
#define VEL_LANDED 2     // m/s
#define AVG_BUFFER_SIZE 6  // size of rolling average buffer

typedef enum {
    FP_INIT,
    FP_READY,  // on the pad
    FP_BOOST,
    FP_FAST,  // faster than mach 1
    FP_COAST,
    FP_DROGUE,  // trigger drogue
    FP_MAIN,    // trigger chute
    FP_LANDED,
} FlightPhase;

typedef struct {
    // x is up!
    float time;          // seconds
    Vector posNED;       // m
    Vector velNED;       // m/s
    Vector accNED;       // m/s^2
    Vector velBody;      // m/s
    Vector accBody;      // m/s^2
    Vector orientation;  // deg ??
} StateEst;

SensorData sensorFrame2SensorData(SensorFrame frame);

/**
 * @brief
 *
 * @param data data, should change this back to SensorData not Sensor Frame
 * eventually
 * @param s_flight_phase
 * @param currentState current state
 * @param imu_up unit vector representing the up direction of IMU accel in
 * body (z up) frame
 * @param high_g_up unit vector representing the up direction of high g accel in
 * body (z up) frame
 * @param acc_buffer buffer for accel rolling average
 * @param baro_buffer buffer for baro rolling average
 */
void fp_update(SensorFrame* data, FlightPhase* s_flight_phase,
               StateEst* currentState, Vector imu_up, Vector high_g_up,
               float* acc_buffer, float* baro_buffer);

StateEst zeroState();

/**
 * @brief
 *
 * @param new_value
 * @param buffer
 * @param buffer_size
 * @return float
 */
float rolling_average(float new_value, float* buffer, int buffer_size);

#endif  // FLIGHT_ESTIMATION_H