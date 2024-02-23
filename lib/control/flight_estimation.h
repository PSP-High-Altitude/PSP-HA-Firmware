#ifndef FLIGHT_ESTIMATION_H
#define FLIGHT_ESTIMATION_H

#include <stdio.h>

#include "data.h"
#include "max_m10s.h"
#include "vector.h"

// set these befire flight
#define HEIGHT_MAIN 300            // m
#define INIT_TIME 10               // s
#define ACC_BOOST 50               // m/s^2
#define ACC_COAST 0                // m/s^2
#define VEL_FAST 250               // m/s
#define IMU_UP 2                   // +/- 1, 2, 3 for x, y, z
#define HIGH_G_UP 2                // +/- 1, 2, 3 for x, y, z
#define LOW_G_CUTOFF 14            // g
#define G 9.81                     // m/s^2
#define VEL_DROGUE 0               // m/s
#define VEL_LANDED 2               // m/s
#define HEIGHT_LANDED 50           // m
#define GPS_ACCURACY_LIMIT_POS 20  // m
#define GPS_ACCURACY_LIMIT_VEL 5   // m/s
#define AVERAGING_PERIOD_MS 100    // period of rolling average buffer
#define DROGUE_LOCKOUT_MS 10000  // time after launch to lock out drogue deploy

typedef struct {
    float* buffer;
    float sum;
    uint16_t count;
    uint16_t index;
    uint16_t size;
} AverageBuffer;

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
 */
void fp_init(FlightPhase* s_flight_phase, StateEst* current_state,
             Vector* imu_up, Vector* high_g_up, AverageBuffer* acc_buffer,
             AverageBuffer* baro_buffer);

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
 */
void fp_update(SensorFrame* data, GPS_Fix_TypeDef* gps,
               FlightPhase* s_flight_phase, StateEst* current_state,
               Vector* imu_up, Vector* high_g_up, AverageBuffer* acc_buffer,
               AverageBuffer* baro_buffer);

StateEst zeroState();

/**
 * @brief
 *
 * @param new_value value to push into the average buffer
 * @param buffer buffer to push the value into
 */
void push_to_average(float new_value, AverageBuffer* buffer);

#endif  // FLIGHT_ESTIMATION_H