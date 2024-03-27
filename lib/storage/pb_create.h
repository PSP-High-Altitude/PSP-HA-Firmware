#ifndef PB_CREATE_H
#define PB_CREATE_H

#include "gps.pb.h"
#include "pb_encode.h"
#include "sensor.pb.h"
#include "state.pb.h"

#define SENSOR_BUF_LEN 256
#define GPS_BUF_LEN 256
#define STATE_BUF_LEN 256

pb_byte_t* create_sensor_buffer(SensorFrame* frame, size_t* size);
pb_byte_t* create_gps_buffer(GpsFrame* frame, size_t* size);
pb_byte_t* create_state_buffer(StateFrame* frame, size_t* size);

#endif  // PB_CREATE_H