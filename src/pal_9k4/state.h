#ifndef STATE_H
#define STATE_H

#include "flight_estimation.h"
#include "max_m10s.h"
#include "sensor.pb.h"
#include "status.h"

#define MAX_POLLING_PERIOD_MS 5  // 200 Hz
#define MAX_AVG_BUFFER_SIZE (AVERAGING_PERIOD_MS / MAX_POLLING_PERIOD_MS)

#define SENSOR_UPDATE_TIMEOUT_MS 1000

Status init_state_est();
Status reset_state_est();

void update_latest_sensor_frame(SensorFrame* sensor_frame);
void update_latest_gps_fix(GPS_Fix_TypeDef* gps_fix);

FlightPhase* get_last_flight_phase();

void state_est_task();

#endif  // STATE_H
