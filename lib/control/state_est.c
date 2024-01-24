#include "state_est.h"
// Not using this anymore
#include "flight_logic.h"

void state_est_main() {
    FlightPhase phase;
    StateEst state;
    SensorData data;

    while (1) {
        fp_update(&data, &phase, &state);
        printf("phase: %d", phase);
    }
}
// StateEst update_state(SensorData* data, GpsData* gpsData, FlightPhase*
// s_flight_phase,
//                       StateEst* currentState) {
//     // TODO: FLIGHT LOGIC GOES HERE
//     switch (*s_flight_phase) {
//         case FP_INIT:
//             s_flight_phase = fp_init();
//             break;
//         case FP_READY:
//             s_flight_phase = fp_ready();
//             break;
//         case FP_BOOST:
//             s_flight_phase = fp_boost();
//             break;
//         case FP_FAST:
//             s_flight_phase = fp_fast();
//             break;
//         case FP_COAST:
//             s_flight_phase = fp_coast();
//             break;
//         case FP_DROGUE:
//             s_flight_phase = fp_descent();
//             break;
//         case FP_MAIN:
//             s_flight_phase = fp_final();
//             break;
//         case FP_LANDED:
//             s_flight_phase = fp_landed();
//             break;
//         default:
//             s_flight_phase = fp_init();
//             break;
//     }
// }