#include "flight_estimation.h"

#include "accel_est.h"
#include "pressure_altitude.h"

void fp_init(FlightPhase* s_flight_phase, StateEst* current_state,
             Vector* imu_up, Vector* high_g_up, float* acc_buffer,
             float* baro_buffer, uint16_t buffer_size) {
    *s_flight_phase = FP_INIT;
    *current_state = zeroState();
    switch (IMU_UP) {
        case -1:
            *imu_up = newVec(-1, 0, 0);
            break;
        case 1:
            *imu_up = newVec(1, 0, 0);
            break;
        case -2:
            *imu_up = newVec(0, -1, 0);
            break;
        case 2:
            *imu_up = newVec(0, 1, 0);
            break;
        case -3:
            *imu_up = newVec(0, 0, -1);
            break;
        case 3:
            *imu_up = newVec(0, 0, 1);
            break;
        default:
            break;
    }
    switch (HIGH_G_UP) {
        case -1:
            *high_g_up = newVec(-1, 0, 0);
            break;
        case 1:
            *high_g_up = newVec(1, 0, 0);
            break;
        case -2:
            *high_g_up = newVec(0, -1, 0);
            break;
        case 2:
            *high_g_up = newVec(0, 1, 0);
            break;
        case -3:
            *high_g_up = newVec(0, 0, -1);
            break;
        case 3:
            *high_g_up = newVec(0, 0, 1);
            break;
        default:
            break;
    }
    memset(acc_buffer, 0, buffer_size * sizeof(float));
    memset(baro_buffer, 0, buffer_size * sizeof(float));
}

void fp_update(SensorFrame* data, GPS_Fix_TypeDef* gps,
               FlightPhase* s_flight_phase, StateEst* current_state,
               Vector* imu_up, Vector* high_g_up, float* acc_buffer,
               float* baro_buffer, uint16_t buffer_size) {
    Vector new_accel;
    // Vector g_vec = vscale();  // TODO: double check this
    SensorData vecData = sensorFrame2SensorData(*data);
    float dt;
    float h0 = 0;        // initial height m ASL
    float gps_h0 = 0;    // initial gps height m MSL
    float a_up_avg = 0;  // up acceleration from rolling average
    float x_up_avg = 0;  // from pressure, hieght ASL

    // z up in state
    // x and y are not in the right place but it's fine for now
    // I will redo this whole conversion system later

    // Check validity of GPS and Sensor data
    uint8_t valid_gps = gps->fix_valid && !gps->invalid_llh &&
                        gps->num_sats >= 4 &&
                        gps->accuracy_horiz < GPS_ACCURACY_LIMIT_POS &&
                        gps->accuracy_vertical < GPS_ACCURACY_LIMIT_POS &&
                        gps->accuracy_speed < GPS_ACCURACY_LIMIT_VEL;
    uint8_t valid_sensor = !isnan(vecData.accel.x) && !isnan(vecData.accel.y) &&
                           !isnan(vecData.accel.z) && !isnan(vecData.acch.x) &&
                           !isnan(vecData.acch.y) && !isnan(vecData.acch.z) &&
                           !isnan(vecData.gyro.x) && !isnan(vecData.gyro.y) &&
                           !isnan(vecData.gyro.z) && !isnan(vecData.mag.x) &&
                           !isnan(vecData.mag.y) && !isnan(vecData.mag.z) &&
                           !isnan(vecData.temperature) &&
                           !isnan(vecData.pressure);
    // Also check that not all fields are zero
    valid_sensor =
        valid_sensor &&
        !((vnorm(vecData.accel) == 0) && (vnorm(vecData.acch) == 0) &&
          (vnorm(vecData.gyro) == 0) && (vnorm(vecData.mag) == 0) &&
          (vecData.temperature == 0) && (vecData.pressure == 0));

    // set up to z and adjust for g
    Vector bodyUp = newVec(0, 0, 1);  // z up
    if (vnorm(vecData.accel) < LOW_G_CUTOFF) {
        new_accel.x = data->acc_i_x;
        new_accel.y = data->acc_i_y;
        new_accel.z = vdot(vecData.accel, *imu_up) - 1;
    } else {
        new_accel.x = data->acc_i_x;
        new_accel.y = data->acc_i_y;
        new_accel.z = vdot(vecData.acch, *high_g_up) - 1;
    }
    current_state->accBody = vscale(new_accel, G);  // convert to m/s^2

    // time
    dt = (data->timestamp * 1E-6) - current_state->time;
    current_state->time = data->timestamp * 1E-6;  // convert us to s
    switch (*s_flight_phase) {
        case FP_INIT:
            // s_flight_phase = fp_init();
            // z is up!!!
            // adjust for gravity
            if (current_state->time > 0) {
                *s_flight_phase = FP_READY;
            }
            break;
        case FP_READY:
            if (valid_gps) {
                gps_h0 = gps->height_msl;  // set gps initial height
            }
            if (valid_sensor) {
                h0 = rolling_average(pressureToAltitude(data->pressure),
                                     baro_buffer,
                                     buffer_size);  // set initial height
            }

            a_up_avg =
                rolling_average(vnorm(current_state->accBody), acc_buffer,
                                buffer_size);  // this one is norm in case I
                                               // messed up the up direction
            if (a_up_avg > ACC_BOOST) {
                // using norm for now in case the up direction is messed up
                // snomehow. Should us z/up in the future
                *s_flight_phase = FP_BOOST;
            }
            break;
        case FP_BOOST:
            update_accel_est(current_state, dt, bodyUp);
            a_up_avg = rolling_average(current_state->accBody.z, acc_buffer,
                                       buffer_size);  // now use up (z)
            if ((a_up_avg < ACC_COAST) &&
                (current_state->velNED.z * -1 < VEL_FAST)) {
                *s_flight_phase = FP_FAST;
            } else if (a_up_avg < ACC_COAST) {
                *s_flight_phase = FP_COAST;
            }
            break;
        case FP_FAST:
            update_accel_est(current_state, dt, bodyUp);
            if (current_state->velNED.z * -1 <= VEL_FAST) {
                *s_flight_phase = FP_COAST;
            }
            break;
        case FP_COAST:
            update_accel_est(current_state, dt, bodyUp);
            if (current_state->velNED.z * -1 >= DROGUE_V) {
                *s_flight_phase = FP_DROGUE;
            }
            break;
        case FP_DROGUE:
            current_state->posNED.z =
                -1 * (pressureToAltitude(data->pressure) - h0);
            x_up_avg =
                rolling_average(current_state->posNED.z * -1, baro_buffer,
                                buffer_size);  // average pressure height
            if (x_up_avg <= MAIN_HEIGHT) {
                *s_flight_phase = FP_MAIN;
            }
            break;
        case FP_MAIN:
            current_state->posNED.z =
                -1 * (pressureToAltitude(data->pressure) - h0);
            a_up_avg = rolling_average(current_state->accBody.z, acc_buffer,
                                       buffer_size);  // now use up (z)
            x_up_avg =
                rolling_average(current_state->posNED.z * -1, baro_buffer,
                                buffer_size);  // average pressure height
            if ((vnorm(current_state->accBody) <= (2 * G)) &&
                ((current_state->velNED.z * -1) < VEL_LANDED)) {
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