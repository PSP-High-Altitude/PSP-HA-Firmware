#ifndef FLIGHT_ESTIMATION_H
#define FLIGHT_ESTIMATION_H

#include <stdio.h>

#include "flight_logic.h"
#include "state_est.h"
#define MAINHEIGHT 100

//

// Status fp_update(SensorData* data, GpsData* gpsData,
//                 FlightPhase* s_flight_phase, StateEst* currentState) {
SensorData sensorFrame2SensorData(SensorFrame frame);
void fp_update(SensorData data, FlightPhase* s_flight_phase,
               StateEst* currentState);
#endif  // FLIGHT_ESTIMATION_H