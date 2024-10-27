#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "data.h"
#include "status.h"

typedef enum {
    FP_INIT,
    FP_READY,  // on the pad
    FP_BOOST_1,
    FP_COAST_1,
    FP_STAGE,
    FP_IGNITE,
    FP_BOOST_2,
    FP_COAST_2,
    FP_DROGUE,  // trigger drogue
    FP_MAIN,    // trigger chute
    FP_LANDED,
} FlightPhase;

typedef enum { FP_STG_GO, FP_STG_NOGO, FP_STG_WAIT } FlightStageStatus;

Status fp_init();

FlightPhase fp_get();

Status fp_update(const SensorFrame* sensor_frame);

#endif  // FLIGHT_CONTROL_H
