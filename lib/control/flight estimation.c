
#include "flight_logic.h"
#include "state_est.h"
#define MAINHEIGHT 100

// Status fp_update(SensorData* data, GpsData* gpsData,
//                 FlightPhase* s_flight_phase, StateEst* currentState) {
Status fp_update(SensorData* data, FlightPhase* s_flight_phase,
                 StateEst* currentState) {
    float ax = currentState->accBody.x;
    float az = currentState->accBody.z;
    switch (*s_flight_phase) {
        case FP_INIT:
            // s_flight_phase = fp_init();
            // x is up!!!
            break;
        case FP_READY:
            break;
        case FP_BOOST:
            if ((ax <= 300) && (ax >= 200)) {
                s_flight_phase = FP_FAST;
            }
            break;
        case FP_FAST:
            if (ax <= 200) {
                s_flight_phase = FP_COAST;
            }
            break;
        case FP_COAST:
            if (az >= MAINHEIGHT) {
                if (az <= 40) {
                    s_flight_phase = FP_DROGUE;
                }
            }
            break;
        case FP_DROGUE:
            if (az <= MAINHEIGHT) {
                s_flight_phase = FP_MAIN;
            }
            break;
        case FP_MAIN:
            if (az <= 50) {
                s_flight_phase = FP_LANDED;
            }
            break;
        case FP_LANDED:
            s_flight_phase = fp_landed();
            break;
        default:
            s_flight_phase = fp_init();
            break;
    }
}