#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <stdio.h>

#include "data.h"
#include "flight_control.h"
#include "status.h"
#include "vector.h"

typedef struct {
    // x is up!
    float time;          // seconds
    Vector posNED;       // m
    Vector velNED;       // m/s
    Vector accNED;       // m/s^2
    Vector velBody;      // m/s
    Vector accBody;      // m/s^2
    Vector orientation;  // deg ??
} StateEst;

Status se_init();

const StateEst* se_predict();

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame);

#endif  // STATE_ESTIMATION_H
