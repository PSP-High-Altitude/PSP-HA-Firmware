#ifndef FLIGHT_ESTIMATION_H
#define FLIGHT_ESTIMATION_H

#include <stdio.h>

#include "state_est.h"

#define MAIN_HEIGHT 100  // m
#define INIT_TIME 10     // s
#define ACC_BOOST 20     // m/s^2
#define VEL_FAST 300     // m/s

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

SensorData sensorFrame2SensorData(SensorFrame frame);
void fp_update(SensorFrame* data, FlightPhase* s_flight_phase,
               StateEst* currentState);
#endif  // FLIGHT_ESTIMATION_H