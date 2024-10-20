#include "state_estimation.h"

#include <malloc.h>
#include <math.h>

#include "quat.h"
#include "vector.h"

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

#define G_MAG 9.81f

typedef Vector* (*OrientFunc)(float, float, float, Vector*);
Status init_buffer(VecBuffer* buffer, size_t buffer_size);
VecBuffer* update_buffer(VecBuffer* buffer, float vec_x, float vec_y,
                         float vec_z);
Vector* sensor_convert_x_up(float sensor_x, float sensor_y, float sensor_z,
                            Vector* v_out);
Vector* sensor_convert_y_up(float sensor_x, float sensor_y, float sensor_z,
                            Vector* v_out);
Vector* sensor_convert_z_up(float sensor_x, float sensor_y, float sensor_z,
                            Vector* v_out);
Vector* sensor_convert_x_down(float sensor_x, float sensor_y, float sensor_z,
                              Vector* v_out);
Vector* sensor_convert_y_down(float sensor_x, float sensor_y, float sensor_z,
                              Vector* v_out);
Vector* sensor_convert_z_down(float sensor_x, float sensor_y, float sensor_z,
                              Vector* v_out);
OrientFunc get_orientation_function(SensorDirection up_dir);

StateEst* s_current_state = {0};

VecBuffer* s_acc_h_buffer = {0};
VecBuffer* s_acc_i_buffer = {0};
VecBuffer* s_baro_buffer = {0};

OrientFunc orientation_function = NULL;

float s_dt_prev = 0;

Status se_init() {
    orientation_function = get_orientation_function(DEFAULT_ORIENTATION);
    Status status;
    status = init_buffer(s_acc_h_buffer, STATE_EST_BUFFERS_SIZE);
    if (status != STATUS_OK) {
        return status;
    }
    status = init_buffer(s_acc_i_buffer, STATE_EST_BUFFERS_SIZE);
    if (status != STATUS_OK) {
        return status;
    }
    status = init_buffer(s_baro_buffer, STATE_EST_BUFFERS_SIZE);
    if (status != STATUS_OK) {
        return status;
    }
    return STATUS_OK;
}

const StateEst* se_predict() { return s_current_state; }

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame) {
    float acc_h_x, acc_h_y, acc_h_z;
    float acc_i_x, acc_i_y, acc_i_z;
    float gyro_x, gyro_y, gyro_z;
    float baro;
    float t;
    float dt;

    if (sensor_frame == NULL) {
        t = s_current_state->time + s_dt_prev;
        dt = s_dt_prev;

        acc_h_x = s_current_state->accBody.x;
        acc_h_y = s_current_state->accBody.y;
        acc_h_z = s_current_state->accBody.z;

        acc_i_x = s_current_state->accBody.x;
        acc_i_y = s_current_state->accBody.y;
        acc_i_z = s_current_state->accBody.z;

        gyro_x = s_current_state->accBody.x;
        gyro_y = s_current_state->accBody.y;
        gyro_z = s_current_state->accBody.z;

        baro = s_current_state->posBody.x;
    } else {
        t = sensor_frame->timestamp / 1e-6f;
        dt = t - s_current_state->time;
        s_dt_prev = dt;

        acc_h_x = isnan(sensor_frame->acc_h_x) ? s_current_state->accBody.x
                                               : sensor_frame->acc_h_x * G_MAG;
        acc_h_y = isnan(sensor_frame->acc_h_y) ? s_current_state->accBody.y
                                               : sensor_frame->acc_h_y * G_MAG;
        acc_h_z = isnan(sensor_frame->acc_h_z) ? s_current_state->accBody.z
                                               : sensor_frame->acc_h_z * G_MAG;

        acc_i_x = isnan(sensor_frame->acc_i_x) ? s_current_state->accBody.x
                                               : sensor_frame->acc_i_x * G_MAG;
        acc_i_y = isnan(sensor_frame->acc_i_y) ? s_current_state->accBody.y
                                               : sensor_frame->acc_i_y * G_MAG;
        acc_i_z = isnan(sensor_frame->acc_i_z) ? s_current_state->accBody.z
                                               : sensor_frame->acc_i_z * G_MAG;

        gyro_x = isnan(sensor_frame->rot_i_x)
                     ? s_current_state->angVelBody.x
                     : sensor_frame->rot_i_x * 180.0f / M_PI;
        gyro_y = isnan(sensor_frame->rot_i_y)
                     ? s_current_state->angVelBody.y
                     : sensor_frame->rot_i_y * 180.0f / M_PI;
        gyro_z = isnan(sensor_frame->rot_i_z)
                     ? s_current_state->angVelBody.z
                     : sensor_frame->rot_i_z * 180.0f / M_PI;

        baro =
            (isnan(sensor_frame->pressure) || isnan(sensor_frame->temperature))
                ? s_current_state->posBody.x
                : BARO_ALT(sensor_frame->pressure);
    }

    Vector acc_old = s_current_state->accBody;
    Vector iacc_old = s_current_state->accGeo;
    Vector vel_old = s_current_state->velBody;
    Vector ivel_old = s_current_state->velGeo;

    update_buffer(s_acc_h_buffer, acc_h_x, acc_h_y, acc_h_z);
    update_buffer(s_acc_i_buffer, acc_i_x, acc_i_y, acc_i_z);
    update_buffer(s_baro_buffer, baro, s_current_state->posBody.y,
                  s_current_state->posBody.z);

    if (fabsf(acc_h_x) > LOW_G_MAX_ACC || fabsf(acc_h_y) > LOW_G_MAX_ACC ||
        fabsf(acc_h_z) > LOW_G_MAX_ACC) {
        vec_copy(&(s_acc_h_buffer->avg), &(s_current_state->accBody));
    } else {
        vec_copy(&(s_acc_i_buffer->avg), &(s_current_state->accBody));
    }
    Vector vec_temp;
    Quaternion quat_temp;

    Vector ang_vel = {.x = gyro_x, .y = gyro_y, .z = gyro_z};

    quat_rot_inv(&(s_current_state->accBody), &(s_current_state->orientation),
                 &(s_current_state->accGeo));

    vec_int_step(&(s_current_state->velBody), &acc_old,
                 &(s_current_state->accBody), dt, &vec_temp);
    vec_copy(&vec_temp, &(s_current_state->velBody));

    vec_int_step(&(s_current_state->velGeo), &iacc_old,
                 &(s_current_state->accGeo), dt, &vec_temp);
    vec_copy(&vec_temp, &(s_current_state->velGeo));

    if (CHUTE_DEPLOYED(phase)) {
        vec_copy(&(s_baro_buffer->avg), &(s_current_state->posBody));
    } else {
        vec_int_step(&(s_current_state->posBody), &vel_old,
                     &(s_current_state->velBody), dt, &vec_temp);
        vec_copy(&vec_temp, &(s_current_state->posBody));
    }
    vec_int_step(&(s_current_state->posGeo), &ivel_old,
                 &(s_current_state->posGeo), dt, &vec_temp);
    vec_copy(&vec_temp, &(s_current_state->posGeo));

    quat_step(&(s_current_state->orientation), &ang_vel, dt, &quat_temp);
    quat_copy(&quat_temp, &(s_current_state->orientation));
    return STATUS_OK;
}

Status init_buffer(VecBuffer* buffer, size_t buffer_size) {
    buffer->vectors = malloc(sizeof(Vector) * buffer_size);
    if (buffer->vectors == NULL) {
        return STATUS_MEMORY_ERROR;
    }
    buffer->current = buffer->vectors + 1;
    buffer->previous = buffer->vectors;
    buffer->i_prev = 0;
    return STATUS_OK;
}

VecBuffer* update_buffer(VecBuffer* buffer, float vec_x, float vec_y,
                         float vec_z) {
    vec_isub(&(buffer->avg), s_acc_i_buffer->current);
    orientation_function(vec_x, vec_y, vec_z, buffer->current);
    vec_iadd(&(buffer->avg), buffer->current);
    buffer->previous = buffer->vectors + buffer->i_prev;
    buffer->current = buffer->vectors + ((buffer->i_prev + 1) % buffer->size);
    buffer->i_prev = (buffer->i_prev + 1) % buffer->size;
    return buffer;
}

OrientFunc get_orientation_function(SensorDirection up_dir) {
    switch (up_dir) {
        case IMU_X_UP:
            return &sensor_convert_x_up;
        case IMU_Y_UP:
            return &sensor_convert_y_up;
        case IMU_Z_UP:
            return &sensor_convert_z_up;
        case IMU_X_DOWN:
            return &sensor_convert_x_down;
        case IMU_Y_DOWN:
            return &sensor_convert_y_down;
        case IMU_Z_DOWN:
            return &sensor_convert_z_down;
        default:
            return &sensor_convert_y_up;
    }
}

Vector* sensor_convert_x_up(float sensor_x, float sensor_y, float sensor_z,
                            Vector* v_out) {
    v_out->x = sensor_x;
    v_out->y = sensor_y;
    v_out->z = sensor_z;
    return v_out;
}

Vector* sensor_convert_y_up(float sensor_x, float sensor_y, float sensor_z,
                            Vector* v_out) {
    v_out->x = sensor_y;
    v_out->y = sensor_z;
    v_out->z = sensor_x;
    return v_out;
}

Vector* sensor_convert_z_up(float sensor_x, float sensor_y, float sensor_z,
                            Vector* v_out) {
    v_out->x = sensor_z;
    v_out->y = sensor_x;
    v_out->z = sensor_y;
    return v_out;
}

Vector* sensor_convert_x_down(float sensor_x, float sensor_y, float sensor_z,
                              Vector* v_out) {
    v_out->x = -sensor_x;
    v_out->y = -sensor_y;
    v_out->z = -sensor_z;
    return v_out;
}

Vector* sensor_convert_y_down(float sensor_x, float sensor_y, float sensor_z,
                              Vector* v_out) {
    v_out->x = -sensor_y;
    v_out->y = -sensor_z;
    v_out->z = -sensor_x;
    return v_out;
}

Vector* sensor_convert_z_down(float sensor_x, float sensor_y, float sensor_z,
                              Vector* v_out) {
    v_out->x = -sensor_z;
    v_out->y = -sensor_x;
    v_out->z = -sensor_y;
    return v_out;
}