#ifndef FLIGHT_LOGIC_H
#define FLIGHT_LOGIC_H

#include "status.h"

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

// TODO: put transition constants here

// function declarations
// Status fp_update();
// FlightPhase fp_init();
// FlightPhase fp_ready();
// FlightPhase fp_boost();
// FlightPhase fp_coast();
// FlightPhase fp_drogue();
// FlightPhase fp_main();
// FlightPhase fp_landed();

#endif  // FLIGHT_LOGIC_H
