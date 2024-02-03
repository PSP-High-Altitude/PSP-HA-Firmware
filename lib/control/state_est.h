#ifndef STATE_EST_H
#define STATE_EST_H

#include "data.h"
#include "vector.h"

// typedef struct Vector {  // this struct should probably go somehwere else
//     float x;
//     float y;
//     float z;
// } Vector;

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

StateEst zeroState();

// StateEst update_state(SensorData* data, FlightPhase* fp,
//                       StateEst* currentState);

#endif  // STATE_EST_H
