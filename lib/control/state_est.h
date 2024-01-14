#ifndef STATE_EST_H
#define STATE_EST_H

#include "data.h"

typedef struct {
    float altitude;      // altitude above launch level (m)
    float vspeed;        // speed in the up direction   (m/s)
    float acceleration;  // magnitude of acceleration   (m/s^2)
} EstState;

EstState update_state(SensorData* data);

#endif  // STATE_EST_H
