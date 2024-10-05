#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "data.h"
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

Status fp_init();

Status fp_update(const SensorFrame* sensor_frame);

FlightPhase fp_update_init(const SensorFrame* sensor_frame);
FlightPhase fp_update_ready(const SensorFrame* sensor_frame);
FlightPhase fp_update_boost(const SensorFrame* sensor_frame);

#endif  // FLIGHT_CONTROL_H