#ifndef HWIL_H
#define HWIL_H

#include "max_m10s.h"
#include "sensor.pb.h"
#include "status.h"

Status get_hwil_sensor_frame(SensorFrame* sensor_frame);
Status get_hwil_gps_fix(GPS_Fix_TypeDef* gps_fix);

#endif  // HWIL_H
