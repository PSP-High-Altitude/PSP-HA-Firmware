#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include "flight_control.h"
#include "quat.h"
#include "sensor.pb.h"
#include "state.pb.h"
#include "status.h"
#include "vector.h"

#ifndef LOW_G_MAX_ACC
#define LOW_G_MAX_ACC (15)
#endif

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

#define G_MAG 9.81f

#define BARO_ALT_BUFFER_SIZE (80)
#define BARO_VEL_BUFFER_SIZE (20)

typedef struct {
    // x is up!
    float time;          // seconds
    float posVert;       // m
    float velVert;       // m/s
    float accVert;       // m/s^2
    Vector posGeo;       // m
    Vector velGeo;       // m/s
    Vector accGeo;       // m/s^2
    Vector angVelBody;   // rad/s
    Quaternion orientation;  // rad (ish)
} StateEst;

typedef enum {
    IMU_X_UP,
    IMU_Y_UP,
    IMU_Z_UP,
    IMU_X_DOWN,
    IMU_Y_DOWN,
    IMU_Z_DOWN
} SensorDirection;

#define DEFAULT_ORIENTATION (IMU_Z_UP)

Status se_init();

Status se_reset();

Status se_set_time(float t_s);

Status se_set_ground_pressure(float p_mbar);

const StateEst* se_predict();

StateFrame se_as_frame();

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame);

#endif  // STATE_ESTIMATION_H
