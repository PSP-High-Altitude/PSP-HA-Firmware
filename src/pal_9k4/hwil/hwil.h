#ifndef HWIL_H
#define HWIL_H

#include "gps.pb.h"
#include "max_m10s.h"
#include "sensor.pb.h"
#include "status.h"

extern const size_t HWIL_SENSOR_DATA_SIZE;
extern const SensorFrame HWIL_SENSOR_DATA[];

extern const size_t HWIL_GPS_DATA_SIZE;
extern const GpsFrame HWIL_GPS_DATA[];

Status get_hwil_sensor_frame(SensorFrame* sensor_frame);
Status get_hwil_gps_fix(GPS_Fix_TypeDef* gps_fix);

#endif  // HWIL_H
