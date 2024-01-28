// #include "flight_logic.h"

// Status fp_update() {
//     switch (s_flight_phase) {
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

// things that need to happen in each pahse go here
// return the phase it should be in
// FlightPhase fp_init() {}

// FlightPhase fp_ready() {}

// FlightPhase fp_boost() {}

// FlightPhase fp_coast() {}

// FlightPhase fp_drogue() {}

// FlightPhase fp_main() {}

// FlightPhase fp_landed() {}