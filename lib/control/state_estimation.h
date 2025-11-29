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
#define M_PI (3.1415926535f)
#endif

#ifndef G_MAG
#define G_MAG (9.81f)
#endif

#define BARO_ALT_MEDIAN_WINDOW 25
#define BARO_ALT_SMA_WINDOW 50
#define BARO_VEL_SMA_WINDOW 25
#define BARO_DIFF_WINDOW 5

typedef struct {
    float time;  // seconds

    // Linear model outputs
    float posVert;  // m
    float velVert;  // m/s
    float accVert;  // m/s^2

    // Inertial model outputs
    Vector posGeo;           // m
    Vector velGeo;           // m/s
    Vector accGeo;           // m/s^2
    Vector angVelBody;       // rad/s
    Quaternion orientation;  // rad (ish)

    // EKF outputs
    float posEkf;
    float velEkf;
    float accEkf;

    float orientEkfw;
    float orientEkfx;
    float orientEkfy;
    float orientEkfz;

    float posVarEkf;
    float velVarEkf;
    float accVarEkf;

    float orientVarEkfw;
    float orientVarEkfx;
    float orientVarEkfy;
    float orientVarEkfz;

    // Intermediate variables (NOT LOGGED)
    float posImu;  // m
    float velImu;  // m/s
    float accImu;  // m/s^2

    float posBaro;  // m
    float velBaro;  // m/s
    float accBaro;  // m/s^2
} StateEst;

typedef enum {
    IMU_X_UP,
    IMU_Y_UP,
    IMU_Z_UP,
    IMU_X_DOWN,
    IMU_Y_DOWN,
    IMU_Z_DOWN
} SensorDirection;

Status se_init();

Status se_reset();

Status se_set_time(float t_s);

Status se_set_ground_pressure(float p_mbar);

const StateEst* se_predict();

StateFrame se_as_frame();

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame);

#endif  // STATE_ESTIMATION_H
