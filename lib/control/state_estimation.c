#include "state_estimation.h"

#include <malloc.h>
#include <math.h>

#include "backup.h"
#include "quat.h"
#include "vector.h"

#define MAX(a, b) (a > b ? a : b)
#define DEG_TO_RAD(x) (x * M_PI / 180)

#define CHUTE_DEPLOYED(x) \
    ((x) == FP_DROGUE || (x) == FP_MAIN || (x) == FP_LANDED)

#define p0 (101325)
#define MBAR_TO_PA(P_mbar) (100 * (P_mbar))
#define BARO_ALT(P) (44330 * (1 - powf(((P) / p0), (1 / 5.255f))))

typedef Vector* (*OrientFunc)(float, float, float, Vector*);
static Status init_baro_buffer(BaroBuffer** buffer, size_t buffer_size);
BaroBuffer* update_baro_buffer(BaroBuffer* buffer, float baro);
static Vector* sensor_convert_x_up(float sensor_x, float sensor_y,
                                   float sensor_z, Vector* v_out);
static Vector* sensor_convert_y_up(float sensor_x, float sensor_y,
                                   float sensor_z, Vector* v_out);
static Vector* sensor_convert_z_up(float sensor_x, float sensor_y,
                                   float sensor_z, Vector* v_out);
static Vector* sensor_convert_x_down(float sensor_x, float sensor_y,
                                     float sensor_z, Vector* v_out);
static Vector* sensor_convert_y_down(float sensor_x, float sensor_y,
                                     float sensor_z, Vector* v_out);
static Vector* sensor_convert_z_down(float sensor_x, float sensor_y,
                                     float sensor_z, Vector* v_out);
static OrientFunc get_orientation_function(SensorDirection up_dir);

static StateEst* s_current_state = NULL;

static BaroBuffer* s_baro_buffer = NULL;

OrientFunc orientation_function = NULL;

Vector s_grav_vec = {.x = -G_MAG, .y = 0.0f, .z = 0.0f};

Status se_init() {
    orientation_function = get_orientation_function(DEFAULT_ORIENTATION);
    EXPECT_OK(init_baro_buffer(&s_baro_buffer, STATE_EST_BUFFERS_SIZE),
              "failed to init stat est buffer\n");
    s_current_state = &(backup_get_ptr()->state_estimate);
    return STATUS_OK;
}

Status se_reset() {
    memset(s_current_state, 0, sizeof(StateEst));
    s_current_state->orientation.w = 1;
    return STATUS_OK;
}

Status se_set_time(float t_s) {
    s_current_state->time = t_s;
    return STATUS_OK;
}

const StateEst* se_predict() { return s_current_state; }

StateFrame se_as_frame() {
    StateFrame frame = {
        // Inertial values
        .pos_geo_x = s_current_state->posGeo.x,
        .pos_geo_y = s_current_state->posGeo.y,
        .pos_geo_z = s_current_state->posGeo.z,

        .vel_geo_x = s_current_state->velGeo.x,
        .vel_geo_y = s_current_state->velGeo.y,
        .vel_geo_z = s_current_state->velGeo.z,

        .acc_geo_x = s_current_state->accGeo.x,
        .acc_geo_y = s_current_state->accGeo.y,
        .acc_geo_z = s_current_state->accGeo.z,

        // Body values
        .pos_body_x = s_current_state->posBody.x,
        .pos_body_y = s_current_state->posBody.y,
        .pos_body_z = s_current_state->posBody.z,

        .vel_body_x = s_current_state->velBody.x,
        .vel_body_y = s_current_state->velBody.y,
        .vel_body_z = s_current_state->velBody.z,

        .acc_body_x = s_current_state->accBody.x,
        .acc_body_y = s_current_state->accBody.y,
        .acc_body_z = s_current_state->accBody.z,

        .angvel_body_x = s_current_state->angVelBody.x,
        .angvel_body_y = s_current_state->angVelBody.y,
        .angvel_body_z = s_current_state->angVelBody.z,

        // Orientation
        .orientation_w = s_current_state->orientation.w,
        .orientation_x = s_current_state->orientation.x,
        .orientation_y = s_current_state->orientation.y,
        .orientation_z = s_current_state->orientation.z,
    };

    return frame;
}

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame) {
    Vector acc_old = s_current_state->accBody;
    Vector iacc_old = s_current_state->accGeo;
    Vector vel_old = s_current_state->velBody;
    Vector ivel_old = s_current_state->velGeo;

    /** IMPORTANT NOTE
     *
     * In this preamble section, we're setting accBody and angVelBody based off
     * sensor values. It's critical that the state estimation logic does not
     * modify either of these variables after the end of the preamble, because
     * we're assuming that we're able to carry forward old values in case we
     * don't get new ones from the next sensor frame.
     */

    // Sensor timestamp is in us, so convert to seconds (float)
    float t = sensor_frame->timestamp / 1e6;
    float dt = t - s_current_state->time;
    s_current_state->time = t;

    // Decide which acceleration value to use
    if (!isnan(sensor_frame->acc_i_x) &&  ///
        !isnan(sensor_frame->acc_i_y) &&  ///
        !isnan(sensor_frame->acc_i_z)) {
        // If the high-precision acceleration values are valid, prefer those
        orientation_function(sensor_frame->acc_i_x * G_MAG,  ///
                             sensor_frame->acc_i_y * G_MAG,  ///
                             sensor_frame->acc_i_z * G_MAG,  ///
                             &(s_current_state->accBody));
        vec_iadd(&(s_current_state->accBody), &s_grav_vec);

        // Check if we're exceeding the high-precision limit on any axis
        if (fabsf(sensor_frame->acc_i_x) > LOW_G_MAX_ACC ||  ///
            fabsf(sensor_frame->acc_i_y) > LOW_G_MAX_ACC ||  ///
            fabsf(sensor_frame->acc_i_z) > LOW_G_MAX_ACC) {
            // If we are, check if we have valid high-range values
            if (!isnan(sensor_frame->acc_h_x) &&
                !isnan(sensor_frame->acc_h_y) &&
                !isnan(sensor_frame->acc_h_z)) {
                // If we do, then, just use those straight up
                orientation_function(sensor_frame->acc_h_x * G_MAG,  ///
                                     sensor_frame->acc_h_y * G_MAG,  ///
                                     sensor_frame->acc_h_z * G_MAG,  ///
                                     &(s_current_state->accBody));
                vec_iadd(&(s_current_state->accBody), &s_grav_vec);
            } else {
                // If we don't, then we have to make the decision between
                // sticking with the old values or going with the new ones
                if (acc_old.x > s_current_state->accBody.x ||  ///
                    acc_old.y > s_current_state->accBody.y ||  ///
                    acc_old.z > s_current_state->accBody.z) {
                    // If the old values are higher on any axis then the new
                    // values, use those since they're probably more accurate
                    s_current_state->accBody = acc_old;
                }
            }
        }
    } else if (!isnan(sensor_frame->acc_i_x) &&  ///
               !isnan(sensor_frame->acc_i_y) &&  ///
               !isnan(sensor_frame->acc_i_z)) {
        // If the high-precision values are invalid but the high-range ones are
        // valid, not much to think about -- just go with the high-range ones
        orientation_function(sensor_frame->acc_h_x * G_MAG,  ///
                             sensor_frame->acc_h_y * G_MAG,  ///
                             sensor_frame->acc_h_z * G_MAG,  ///
                             &(s_current_state->accBody));
        vec_iadd(&(s_current_state->accBody), &s_grav_vec);
    } else {
        // If neither were valid, then we reuse the acceleration values from
        // last time (which is a no-op since they're already in s_current_state)
        PAL_LOGW("Both accelerometers invalid; reusing old accelerations\n");
    }

    // Update the angular velocity if we have valid IMU data
    if (!isnan(sensor_frame->rot_i_x) &&  ///
        !isnan(sensor_frame->rot_i_y) &&  ///
        !isnan(sensor_frame->rot_i_z)) {
        orientation_function(DEG_TO_RAD(sensor_frame->rot_i_x),  ///
                             DEG_TO_RAD(sensor_frame->rot_i_y),  ///
                             DEG_TO_RAD(sensor_frame->rot_i_z),  ///
                             &(s_current_state->angVelBody));
    }

    // Update the baro altitude buffer if we have a valid pressure value
    if (!isnan(sensor_frame->pressure)) {
        update_baro_buffer(s_baro_buffer, BARO_ALT(sensor_frame->pressure));
    }

    // STATE ESTIMATION START
    Vector vec_temp;
    Quaternion quat_temp;

    quat_rot_inv(&(s_current_state->accBody), &(s_current_state->orientation),
                 &(s_current_state->accGeo));

    vec_iadd(&(s_current_state->accGeo), &s_grav_vec);

    vec_int_step(&(s_current_state->velBody), &acc_old,
                 &(s_current_state->accBody), dt, &vec_temp);
    vec_copy(&vec_temp, &(s_current_state->velBody));

    vec_int_step(&(s_current_state->velGeo), &iacc_old,
                 &(s_current_state->accGeo), dt, &vec_temp);
    vec_copy(&vec_temp, &(s_current_state->velGeo));

    if (CHUTE_DEPLOYED(phase)) {
        s_current_state->posBody.x = s_baro_buffer->avg;
    } else {
        vec_int_step(&(s_current_state->posBody), &vel_old,
                     &(s_current_state->velBody), dt, &vec_temp);
        vec_copy(&vec_temp, &(s_current_state->posBody));
    }
    vec_int_step(&(s_current_state->posGeo), &ivel_old,
                 &(s_current_state->posGeo), dt, &vec_temp);
    vec_copy(&vec_temp, &(s_current_state->posGeo));

    quat_step(&(s_current_state->orientation), &(s_current_state->angVelBody),
              dt, &quat_temp);
    quat_copy(&quat_temp, &(s_current_state->orientation));

    return STATUS_OK;
}

static Status init_baro_buffer(BaroBuffer** buffer, size_t buffer_size) {
    *buffer = malloc(sizeof(BaroBuffer));
    if (buffer == NULL) {
        return STATUS_MEMORY_ERROR;
    }
    (*buffer)->vals = malloc(sizeof(float) * buffer_size);
    if ((*buffer)->vals == NULL) {
        return STATUS_MEMORY_ERROR;
    }
    (*buffer)->current = (*buffer)->vals + 1;
    (*buffer)->previous = (*buffer)->vals;
    (*buffer)->i_prev = 0;
    (*buffer)->filled_elements = 0;
    (*buffer)->size = buffer_size;
    return STATUS_OK;
}

BaroBuffer* update_baro_buffer(BaroBuffer* buffer, float baro) {
    buffer->avg *= buffer->filled_elements;
    buffer->filled_elements = MAX(buffer->filled_elements + 1, buffer->size);
    buffer->avg -= *buffer->current;
    *buffer->current = baro;
    buffer->avg += *buffer->current;
    buffer->avg *= 1.0f / buffer->filled_elements;
    buffer->previous = buffer->vals + buffer->i_prev;
    buffer->current = buffer->vals + ((buffer->i_prev + 1) % buffer->size);
    buffer->i_prev = (buffer->i_prev + 1) % buffer->size;
    return buffer;
}

static OrientFunc get_orientation_function(SensorDirection up_dir) {
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

static Vector* sensor_convert_x_up(float sensor_x, float sensor_y,
                                   float sensor_z, Vector* v_out) {
    v_out->x = sensor_x;
    v_out->y = sensor_y;
    v_out->z = sensor_z;
    return v_out;
}

static Vector* sensor_convert_y_up(float sensor_x, float sensor_y,
                                   float sensor_z, Vector* v_out) {
    v_out->x = sensor_y;
    v_out->y = sensor_z;
    v_out->z = sensor_x;
    return v_out;
}

static Vector* sensor_convert_z_up(float sensor_x, float sensor_y,
                                   float sensor_z, Vector* v_out) {
    v_out->x = sensor_z;
    v_out->y = sensor_x;
    v_out->z = sensor_y;
    return v_out;
}

static Vector* sensor_convert_x_down(float sensor_x, float sensor_y,
                                     float sensor_z, Vector* v_out) {
    v_out->x = -sensor_x;
    v_out->y = -sensor_y;
    v_out->z = -sensor_z;
    return v_out;
}

static Vector* sensor_convert_y_down(float sensor_x, float sensor_y,
                                     float sensor_z, Vector* v_out) {
    v_out->x = -sensor_y;
    v_out->y = -sensor_z;
    v_out->z = -sensor_x;
    return v_out;
}

static Vector* sensor_convert_z_down(float sensor_x, float sensor_y,
                                     float sensor_z, Vector* v_out) {
    v_out->x = -sensor_z;
    v_out->y = -sensor_x;
    v_out->z = -sensor_y;
    return v_out;
}
