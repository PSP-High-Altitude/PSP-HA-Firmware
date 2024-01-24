
#include "flight_logic.h"
#define MAINHEIGHT 100

Status fp_update(SensorData* data, GpsData* gpsData, FlightPhase* s_flight_phase, StateEst* currentState) {
    switch (*s_flight_phase) {
        case FP_INIT:
           // s_flight_phase = fp_init();
            // x is up!!!
            break;
        case FP_READY:
            break;
        case FP_BOOST:
            if ((gx <= 300) && (gx >= 200))
            {
                s_flight_phase = FP_FAST;
            }
            break;
        case FP_FAST:
            if (gx <= 200)
            {
                s_flight_phase = FP_COAST;
            }
            break;
        case FP_COAST:
            if(gz >= MAINHEIGHT)
            {
                if(gz <= 40)
                {
                    s_flight_phase = FP_DROGUE;
                }
            }
            break;
        case FP_DROGUE:
            if( gz <= MAINHEIGHT)
            {
                s_flight_phase = FP_MAIN;
            }
            break;
        case FP_MAIN:
        if( gz <= 50)
        {
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