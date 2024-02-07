#ifndef FLIGHT_ESTIMATION_H
#define FLIGHT_ESTIMATION_H

#include <stdio.h>

#include "data.h"
#include "max_m10s.h"
#include "vector.h"

// set these befire flight
#define MAIN_HEIGHT 300            // m
#define INIT_TIME 10               // s
#define ACC_BOOST 50               // m/s^2
#define ACC_COAST 0                // m/s^2
#define VEL_FAST 200               // m/s
#define IMU_UP -2                  // +/- 1, 2, 3 for x, y, z
#define HIGH_G_UP 2                // +/- 1, 2, 3 for x, y, z
#define LOW_G_CUTOFF 14            // g
#define G 9.81                     // m/s^2
#define DROGUE_V 0                 // m/s
#define VEL_LANDED 2               // m/s
#define GPS_ACCURACY_LIMIT_POS 20  // m
#define GPS_ACCURACY_LIMIT_VEL 5   // m/s
#define AVERAGING_PERIOD_MS 100    // period of rolling average buffer

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
 * @param s_flight_phase current flight phase
 * @param current_state current state
 * @param imu_up unit vector representing the up direction of IMU accel in
 * body (z up) frame
 * @param high_g_up unit vector representing the up direction of high g accel in
 * body (z up) frame
 * @param acc_buffer buffer for rolling average of acceleration
 * @param baro_buffer buffer for rolling average of barometer
 * @param buffer_size size of the rolling average buffers
 */
void fp_init(FlightPhase* s_flight_phase, StateEst* current_state,
             Vector* imu_up, Vector* high_g_up, float* acc_buffer,
             float* baro_buffer, uint16_t buffer_size);

/**
 * @brief
 *
 * @param data sensor data
 * @param gps gps fix
 * @param s_flight_phase current flight phase
 * @param current_state current state
 * @param imu_up unit vector representing the up direction of IMU accel in
 * body (z up) frame
 * @param high_g_up unit vector representing the up direction of high g accel in
 * body (z up) frame
 * @param acc_buffer buffer for rolling average of acceleration
 * @param baro_buffer buffer for rolling average of barometer
 * @param buffer_size size of the rolling average buffers
 */
void fp_update(SensorFrame* data, GPS_Fix_TypeDef* gps,
               FlightPhase* s_flight_phase, StateEst* current_state,
               Vector* imu_up, Vector* high_g_up, float* acc_buffer,
               float* baro_buffer, uint16_t buffer_size);

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