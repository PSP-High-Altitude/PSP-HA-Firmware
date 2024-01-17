#ifndef STATE_EST_H
#define STATE_EST_H

#include "data.h"

typedef struct Vector {  // this struct should probably go somehwere else
    float x;
    float y;
    float z;
} Vector;

typedef struct {
    Vector posNED;
    Vector velNED;
    Vector accNED;
    Vector velBody;
    Vector accBody;
} StateEst;

StateEst update_state(SensorData* data, FlightPhase* fp,
                      StateEst* currentState);

#endif  // STATE_EST_H
