#ifndef ACCEL_EST_H
#define ACCEL_EST_H

#include <stdio.h>

#include "flight_estimation.h"

typedef struct {
    double* times;
    double* AccBodyY;
    double* AccDown;
    double* VelDown;
    double* PosDown;
    int i;
    double g;
    double p;
} accel_est;

void init_accel_est(accel_est* obj, int size);
void update_accel_est(StateEst* state, float dt, Vector up);

#endif  // ACCEL_EST_H