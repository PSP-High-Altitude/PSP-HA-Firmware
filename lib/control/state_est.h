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
    Vector posNED;
    Vector velNED;
    Vector accNED;
    Vector velBody;
    Vector accBody;
    Vector orientation;
} StateEst;

StateEst zeroState() {
    StateEst state;
    Vector zeroVec = {0, 0, 0};

    state.posNED = zeroVec;
    state.velNED = zeroVec;
    state.accNED = zeroVec;
    state.velBody = zeroVec;
    state.accBody = zeroVec;
    state.orientation = zeroVec;
}

// StateEst update_state(SensorData* data, FlightPhase* fp,
//                       StateEst* currentState);

#endif  // STATE_EST_H
