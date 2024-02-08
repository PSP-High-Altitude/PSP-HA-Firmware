#include "flight_estimation.h"

#include "accel_est.h"
#include "pressure_altitude.h"
#include "pyros.h"

#define BUFFER_AVERAGE(buffer) (buffer->sum / buffer->count)
#define BUFFER_FULL(buffer) (buffer->count == buffer->size)

void fp_init(FlightPhase* s_flight_phase, StateEst* current_state,
             Vector* imu_up, Vector* high_g_up, AverageBuffer* acc_buffer,
             AverageBuffer* baro_buffer) {
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
    acc_buffer->count = 0;
    baro_buffer->count = 0;
    acc_buffer->sum = 0.0;
    baro_buffer->sum = 0.0;
    acc_buffer->index = 0;
    baro_buffer->index = 0;
    memset(acc_buffer->buffer, 0, acc_buffer->size * sizeof(float));
    memset(baro_buffer->buffer, 0, baro_buffer->size * sizeof(float));
}

void fp_update(SensorFrame* data, GPS_Fix_TypeDef* gps,
               FlightPhase* s_flight_phase, StateEst* current_state,
               Vector* imu_up, Vector* high_g_up, AverageBuffer* acc_buffer,
               AverageBuffer* baro_buffer) {
    Vector new_accel;
    // Vector g_vec = vscale();  // TODO: double check this
    SensorData vecData = sensorFrame2SensorData(*data);
    float dt;
    static float h0 = 0;        // initial height m ASL
    static float gps_h0 = 0;    // initial gps height m MSL
    float a_up_avg = 0;  // up acceleration from rolling average
    float x_up_avg = 0;  // from pressure, height ASL

    // z up in state
    // x and y are not in the right place but it's fine for now
    // I will redo this whole conversion system later

    // Check validity of GPS and Sensor data
    uint8_t valid_gps = gps->fix_valid && !gps->invalid_llh &&
                        gps->num_sats >= 4 &&
                        gps->accuracy_horiz < GPS_ACCURACY_LIMIT_POS &&
                        gps->accuracy_vertical < GPS_ACCURACY_LIMIT_POS &&
                        gps->accuracy_speed < GPS_ACCURACY_LIMIT_VEL;
    uint8_t valid_acc = !isnan(vecData.accel.x) && !isnan(vecData.accel.y) &&
                        !isnan(vecData.accel.z) && !isnan(vecData.acch.x) &&
                        !isnan(vecData.acch.y) && !isnan(vecData.acch.z);
    uint8_t valid_baro =
        !isnan(vecData.temperature) && !isnan(vecData.pressure);
    // Also check valid_acc not all fields are zero
    valid_acc =
        valid_acc && !((vnorm(vecData.accel) == 0) && (vnorm(vecData.acch)));
    valid_baro =
        valid_baro && !((vecData.temperature == 0) && (vecData.pressure == 0));

    // Only update the state if the sensor data is valid
    if (valid_acc) {
        // set up to z and adjust for g
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

        // push to average buffer
        push_to_average(current_state->accBody.z, acc_buffer);
    } else {
        dt = 0.0;  // data not valid so don't update state estimation
    }

    if (valid_baro) {
        // push to average buffer
        push_to_average(pressureToAltitude(data->pressure), baro_buffer);
    }

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
            if (BUFFER_FULL(baro_buffer)) {
                h0 = BUFFER_AVERAGE(
                    baro_buffer);  // set barometer initial height
            }
            if (BUFFER_FULL(acc_buffer)) {
                a_up_avg = BUFFER_AVERAGE(acc_buffer);
            }
            if (BUFFER_FULL(acc_buffer) && a_up_avg > ACC_BOOST) {
                *s_flight_phase = FP_BOOST;
                printf("BOOST at %lld\n", data->timestamp/1000);
            }
            break;
        case FP_BOOST:
            update_accel_est(current_state, dt);
            a_up_avg = BUFFER_AVERAGE(acc_buffer);
            if (a_up_avg > ACC_BOOST) {
                break;  // still in boost
            }
            if ((current_state->velNED.z * -1 > VEL_FAST) ||
                (valid_gps && (gps->vel_down * -1 > VEL_FAST))) {
                *s_flight_phase = FP_FAST;
                printf("FAST at %lld\n", data->timestamp/1000);
            } else if (a_up_avg < ACC_COAST) {
                *s_flight_phase = FP_COAST;
                printf("COAST at %lld\n", data->timestamp/1000);
            }
            break;
        case FP_FAST:
            update_accel_est(current_state, dt);
            if ((current_state->velNED.z * -1 < VEL_FAST) ||
                (valid_gps && (gps->vel_down * -1 < VEL_FAST))) {
                *s_flight_phase =
                    FP_COAST;  // We are slow enough to be in coast
                    printf("COAST at %lld\n", data->timestamp/1000);
            }
            break;
        case FP_COAST:
            update_accel_est(current_state, dt);
            if ((current_state->velNED.z * -1 < VEL_DROGUE) ||
                (valid_gps && (gps->vel_down * -1 < VEL_DROGUE))) {
                *s_flight_phase = FP_DROGUE;
                fire_pyro(DROGUE_PYRO);  // fire drogue
                printf("DROGUE at %lld\n", data->timestamp / 1000);

                // reset baro buffer, this helps in an edge case
                // where the barometer fails at a altitude below
                // main deployment height, and then gets all the
                // way here. In this case, we don't want to use
                // the old data and deploy the main early.
                baro_buffer->count = 0;
                baro_buffer->sum = 0.0;
                baro_buffer->index = 0;
                memset(baro_buffer->buffer, 0,
                       baro_buffer->size * sizeof(float));
            }
            break;
        case FP_DROGUE:
            if (valid_baro) {
                current_state->posNED.z =
                    -1 * (pressureToAltitude(data->pressure) - h0);
            }
            x_up_avg = BUFFER_AVERAGE(baro_buffer) - h0;
            if ((x_up_avg < HEIGHT_MAIN && BUFFER_FULL(baro_buffer)) ||
                (valid_gps && gps->height_msl - gps_h0 < HEIGHT_MAIN)) {
                *s_flight_phase = FP_MAIN;
                fire_pyro(MAIN_PYRO);  // fire main
                printf("MAIN at %lld\n", data->timestamp / 1000);
            }
            break;
        case FP_MAIN:
            current_state->posNED.z =
                -1 * (pressureToAltitude(data->pressure) - h0);
            x_up_avg = BUFFER_AVERAGE(baro_buffer) - h0;
            if (fabs(x_up_avg) < HEIGHT_LANDED ||
                (valid_gps && fabs(gps->height_msl - gps_h0) < HEIGHT_LANDED) ||
                (valid_gps && fabs(gps->vel_down * -1) < VEL_LANDED)) {
                *s_flight_phase = FP_LANDED;
                printf("LANDED at %lld\n", data->timestamp/1000);
            }
            break;
        case FP_LANDED:
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

void push_to_average(float new_value, AverageBuffer* buffer) {
    // If the buffer is not full, increment the count
    if (buffer->count < buffer->size) {
        buffer->count++;
    } else {
        // If the buffer is full, subtract the oldest value from the sum
        buffer->sum -= buffer->buffer[buffer->index];
    }

    // Add the new value to the sum
    buffer->sum += new_value;

    // Store the new value in the buffer
    buffer->buffer[buffer->index] = new_value;

    // Move to the next index
    buffer->index = (buffer->index + 1) % buffer->size;
}