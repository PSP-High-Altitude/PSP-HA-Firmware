#include "flight_estimation.h"

#include "accel_est.h"
#include "pressure_altitude.h"

void fp_update(SensorFrame* data, FlightPhase* s_flight_phase,
               StateEst* currentState, Vector imu_up, Vector high_g_up,
               float* acc_buffer, float* baro_buffer) {
    Vector new_accel;
    // Vector g_vec = vscale();  // TODO: double check this
    SensorData vecData = sensorFrame2SensorData(*data);
    float dt;
    float h0 = 0;  // initial height m ASL
    float a_up_avg = 0;  // up acceleration from rolling average
    float x_up_avg = 0;  // from pressure, hieght ASL

    // z up in state
    // x and y are not in the right place but it's fine for now
    // I will redo this whole conversion system later

    // set up to z and adjust for g
    Vector bodyUp = newVec(0, 0, 1);  // z up
    if (vnorm(vecData.accel) < LOW_G_CUTOFF) {
        new_accel.x = data->acc_i_x;
        new_accel.y = data->acc_i_y;
        new_accel.z = vdot(vecData.accel, imu_up) - 1;
    } else {
        new_accel.x = data->acc_i_x;
        new_accel.y = data->acc_i_y;
        new_accel.z = vdot(vecData.acch, high_g_up) - 1;
    }
    currentState->accBody = vscale(new_accel, G);  // convert to m/s^2

    // time
    dt = currentState->time - data->timestamp * 1E-6;
    currentState->time = data->timestamp * 1E-6;  // convert us to s
    switch (*s_flight_phase) {
        case FP_INIT:
            // s_flight_phase = fp_init();
            // z is up!!!
            // adjust for gravity
            if (currentState->time > 0) {
                *s_flight_phase = FP_READY;
            }
            break;
        case FP_READY:
            h0 =
                rolling_average(pressureToAltitude(data->pressure), baro_buffer,
                                AVG_BUFFER_SIZE);  // set initial height
            a_up_avg =
                rolling_average(vnorm(currentState->accBody), acc_buffer,
                                AVG_BUFFER_SIZE);  // this one is norm in case I
                                                   // messed up the up direction
            if (a_up_avg > ACC_BOOST) {
                // using norm for now in case the up direction is messed up
                // snomehow. Should us z/up in the future
                *s_flight_phase = FP_BOOST;
            }
            break;
        case FP_BOOST:
            update_accel_est(currentState, dt, bodyUp);
            a_up_avg = rolling_average(currentState->accBody.z, acc_buffer,
                                       AVG_BUFFER_SIZE);  // now use up (z)
            if ((a_up_avg < ACC_COAST) &&
                (currentState->velNED.z * -1 < VEL_FAST)) {
                *s_flight_phase = FP_FAST;
            } else if (a_up_avg < ACC_COAST) {
                *s_flight_phase = FP_COAST;
            }
            break;
        case FP_FAST:
            update_accel_est(currentState, dt, bodyUp);
            if (currentState->velNED.z * -1 <= VEL_FAST) {
                *s_flight_phase = FP_COAST;
            }
            break;
        case FP_COAST:
            update_accel_est(currentState, dt, bodyUp);
            if (currentState->velNED.z * -1 >= DROGUE_V) {
                *s_flight_phase = FP_DROGUE;
            }
            break;
        case FP_DROGUE:
            currentState->posNED.z =
                -1 * (pressureToAltitude(data->pressure) - h0);
            x_up_avg =
                rolling_average(currentState->posNED.z * -1, baro_buffer,
                                AVG_BUFFER_SIZE);  // average pressure height
            if (x_up_avg <= MAIN_HEIGHT) {
                *s_flight_phase = FP_MAIN;
            }
            break;
        case FP_MAIN:
            currentState->posNED.z =
                -1 * (pressureToAltitude(data->pressure) - h0);
            a_up_avg = rolling_average(currentState->accBody.z, acc_buffer,
                                       AVG_BUFFER_SIZE);  // now use up (z)
            x_up_avg =
                rolling_average(currentState->posNED.z * -1, baro_buffer,
                                AVG_BUFFER_SIZE);  // average pressure height
            if ((vnorm(currentState->accBody) <= (2 * G)) &&
                ((currentState->velNED.z * -1) < VEL_LANDED)) {
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

StateEst zeroState() {
    StateEst state;
    Vector zeroVec = {0, 0, 0};

    state.posNED = zeroVec;
    state.velNED = zeroVec;
    state.accNED = zeroVec;
    state.velBody = zeroVec;
    state.accBody = zeroVec;
    state.orientation = zeroVec;
    return state;
}

float rolling_average(float new_value, float* buffer, int buffer_size) {
    static float sum =
        0.0;  // Static variable to store the sum of buffer elements
    static int index =
        0;  // Static variable to keep track of the current index in the buffer
    static int count = 0;  // Static variable to keep track of the number of
                           // elements in the buffer

    // If the buffer is not full, increment the count
    if (count < buffer_size) {
        count++;
    } else {
        // If the buffer is full, subtract the oldest value from the sum
        sum -= buffer[index];
    }

    // Add the new value to the sum
    sum += new_value;

    // Store the new value in the buffer
    buffer[index] = new_value;

    // Move to the next index
    index = (index + 1) % buffer_size;

    // Return the rolling average
    return sum / count;
}