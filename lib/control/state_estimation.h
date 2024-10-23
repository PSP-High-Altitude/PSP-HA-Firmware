#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include "data.h"
#include "flight_control.h"
#include "math.h"
#include "quat.h"
#include "status.h"
#include "vector.h"

#ifndef LOW_G_MAX_ACC
#define LOW_G_MAX_ACC (24)
#endif

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

#define G_MAG 9.81f

#define CHUTE_DEPLOYED(x) \
    ((x) == FP_DROGUE || (x) == FP_MAIN || (x) == FP_LANDED)

#define STATE_EST_BUFFERS_SIZE (10)

#define p0 (101325)
#define MBAR_TO_PA(P_mbar) (100 * (P_mbar))
#define BARO_ALT(P) (44330 * (1 - powf(((P) / p0), (1 / 5.255f))))

typedef struct {
    // x is up!
    float time;          // seconds
    Vector posGeo;       // m
    Vector velGeo;       // m/s
    Vector accGeo;       // m/s^2
    Vector posBody;      // m
    Vector velBody;      // m/s
    Vector accBody;      // m/s^2
    Vector angVelBody;   // rad/s
    Quaternion orientation;  // rad (ish)
} StateEst;

typedef struct {
    Vector* vectors;
    Vector* current;
    Vector* previous;
    Vector avg;
    uint16_t i_prev;
    uint16_t size;
    uint16_t filled_elements;
} VecBuffer;

typedef struct {
    float* vals;
    float* current;
    float* previous;
    float avg;
    uint16_t i_prev;
    uint16_t size;
    uint16_t filled_elements;
} BaroBuffer;

typedef enum {
    IMU_X_UP,
    IMU_Y_UP,
    IMU_Z_UP,
    IMU_X_DOWN,
    IMU_Y_DOWN,
    IMU_Z_DOWN
} SensorDirection;

#define DEFAULT_ORIENTATION (IMU_Y_UP)

Status se_init();

Status se_reset();

const StateEst* se_predict();

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame);

#endif  // STATE_ESTIMATION_H
