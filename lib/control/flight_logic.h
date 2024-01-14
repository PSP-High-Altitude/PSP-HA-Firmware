#ifndef FLIGHT_LOGIC_H
#define FLIGHT_LOGIC_H

#include "status.h"

typedef enum {
    FP_INIT,
    FP_READY,
    FP_BOOST,
    FP_COAST,
    FP_DESCENT,
    FP_FINAL,
    FP_LANDED,
} FlightPhase;

Status fp_update();
FlightPhase fp_init();
FlightPhase fp_ready();
FlightPhase fp_boost();
FlightPhase fp_coast();
FlightPhase fp_descent();
FlightPhase fp_final();
FlightPhase fp_landed();

#endif  // FLIGHT_LOGIC_H
