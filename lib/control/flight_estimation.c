#include "flight_estimation.h"

void fp_update(SensorFrame* data, FlightPhase* s_flight_phase,
               StateEst* currentState) {
    Vector new_accel;
    new_accel.x = data->acc_i_x;
    new_accel.y = data->acc_i_y;
    new_accel.z = data->acc_i_z;
    currentState->accBody = vscale(new_accel, 9.81);
    currentState->time = data->timestamp * 10E-6;
    float ax = currentState->accBody.x;
    float az = currentState->accBody.z;
    switch (*s_flight_phase) {
        case FP_INIT:
            // s_flight_phase = fp_init();
            // x is up!!!
            // adjust for gravity
            if (currentState->time > 0) {
                *s_flight_phase = FP_READY;
            }
            break;
        case FP_READY:
            if (ax > ACC_BOOST) {
                *s_flight_phase = FP_BOOST;
            }
            break;
        case FP_BOOST:
            if ((ax <= 300) && (ax >= 200)) {
                *s_flight_phase = FP_FAST;
            }
            break;
        case FP_FAST:
            if (ax <= 200) {
                *s_flight_phase = FP_COAST;
            }
            break;
        case FP_COAST:
            if (az >= MAIN_HEIGHT) {
                if (az <= 40) {
                    *s_flight_phase = FP_DROGUE;
                }
            }
            break;
        case FP_DROGUE:
            if (az <= MAIN_HEIGHT) {
                *s_flight_phase = FP_MAIN;
            }
            break;
        case FP_MAIN:
            if (az <= 50) {
                *s_flight_phase = FP_LANDED;
            }
            break;
        case FP_LANDED:
            *s_flight_phase = FP_LANDED;
            break;
        default:
            *s_flight_phase = FP_INIT;
            break;
    }
}