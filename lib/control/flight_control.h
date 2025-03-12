#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "data.h"
#include "status.h"

typedef enum {
    FP_INIT,
    FP_WAIT,    // not ready
    FP_READY,   // on the pad
    FP_BOOST,   // motor burning
    FP_COAST,   // waiting for apogee
    FP_DROGUE,  // under drogue
    FP_MAIN,    // under main
    FP_LANDED,
    FP_ERROR,
} FlightPhase;

typedef enum { FP_STG_GO, FP_STG_NOGO, FP_STG_WAIT } FlightStageStatus;

// How many buffered frames to consume each iter of the control loop
// This MUST be strictly greater than 1 to deplete the buffer
#define LD_REPLAY_FRAMES_PER_ITER (2)

Status fp_init();

FlightPhase fp_get();

Status fp_update(const SensorFrame* sensor_frame);

#endif  // FLIGHT_CONTROL_H
