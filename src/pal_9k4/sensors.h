#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"
#include "max_m10s.h"
#include "sensor.pb.h"
#include "status.h"

#define SENSOR_NORMAL_READ_INTERVAL_MS 200
#define SENSOR_SLEEP_READ_INTERVAL_MS 200

#define GPS_NORMAL_READ_INTERVAL_MS 200
#define GPS_SLEEP_READ_INTERVAL_MS 200

Status init_sensors();

void pause_sensors();
void start_sensors();

SensorFrame* get_last_sensor_frame();
GPS_Fix_TypeDef* get_last_gps_fix();

void read_sensors_task();
void read_gps_task();

#endif  // SENSORS_H
