#ifndef ACCEL_EST_H
#define ACCEL_EST_H

#include <stdio.h>

#include "flight_estimation.h"

void update_accel_est(StateEst* state, float dt);

#endif  // ACCEL_EST_H