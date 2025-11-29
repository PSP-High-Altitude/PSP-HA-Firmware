#ifndef TELEM_H
#define TELEM_H

#include "flight_control.h"
#include "max_m10s.h"
#include "sensor.pb.h"

Status telem_init();

void telem_update_sensors(SensorFrame *sensor_frame);
void telem_update_gps(GPS_Fix_TypeDef *gps_fix);
void telem_update_fp(FlightPhase flight_phase);

void task_telem_rx();
void task_telem_tx();

#endif  // TELEM_H
