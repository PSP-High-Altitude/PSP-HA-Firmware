#include "flight_estimation.h"

SensorData sensorFrame2SensorData(SensorFrame frame) {
    SensorData data;
    data.timestamp = frame.timestamp;
    data.accel = newVec(frame.acc_i_x, frame.acc_i_y, frame.acc_i_z);
    data.acch = newVec(frame.acc_h_x, frame.acc_h_y, frame.acc_h_z);
    data.gyro = newVec(frame.rot_i_x, frame.rot_i_y, frame.rot_i_z);
    data.mag = newVec(frame.mag_i_x, frame.mag_i_x, frame.mag_i_x);
    data.temperature = frame.temperature;
    data.pressure = frame.pressure;
    return data;
}

void fp_update(SensorData data, FlightPhase* s_flight_phase,
               StateEst* currentState) {
    currentState->accBody = vscale(data.accel, 9.81);

    float ax = currentState->accBody.x;
    float az = currentState->accBody.z;
    // Accel* aPtr = &data->accel;
    // void* vecPtr = (void*)aPtr;
    // (*currentState).accBody = vscale(createVectorFromStruct(vecPtr), 9.81);
    switch (*s_flight_phase) {
        case FP_INIT:
            // s_flight_phase = fp_init();
            // x is up!!!
            if (data.timestamp > 1000) {
                *s_flight_phase = FP_READY;
            }
            break;
        case FP_READY:
            break;
            if (ax > 15) {
                *s_flight_phase = FP_BOOST;
            }
        case FP_BOOST:
            if ((ax <= 300) && (ax >= 200)) {
                *s_flight_phase = FP_FAST;
            }
            break;
        case FP_FAST:
            if (ax <= 200) {
                *s_flight_phase = FP_COAST;
            }
            break;
        case FP_COAST:
            if (az >= MAINHEIGHT) {
                if (az <= 40) {
                    *s_flight_phase = FP_DROGUE;
                }
            }
            break;
        case FP_DROGUE:
            if (az <= MAINHEIGHT) {
                *s_flight_phase = FP_MAIN;
            }
            break;
        case FP_MAIN:
            if (az <= 50) {
                *s_flight_phase = FP_LANDED;
            }
            break;
        case FP_LANDED:
            *s_flight_phase = FP_LANDED;
            break;
        default:
            *s_flight_phase = FP_INIT;
            break;
    }
}