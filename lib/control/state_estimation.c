#include "state_estimation.h"

#include <malloc.h>
#include <math.h>

#include "backup.h"
#include "quat.h"
#include "sma_buffer.h"
#include "vector.h"

#define DEG_TO_RAD(x) (x * M_PI / 180)

#define CHUTE_DEPLOYED(x) \
    ((x) == FP_DROGUE || (x) == FP_MAIN || (x) == FP_LANDED)

typedef Vector* (*OrientFunc)(float, float, float, Vector*);
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

static StateEst* s_current_state_ptr = NULL;

static SmaFloatBuffer s_baro_alt_buffer;
static SmaFloatBuffer s_baro_vel_buffer;

static OrientFunc orientation_function = NULL;

static Vector s_grav_vec = {.x = -G_MAG, .y = 0.0f, .z = 0.0f};

static float s_ground_baro_alt_m = 0.;

__attribute__((unused)) static float se_baro_weight(float alt_m,
                                                    float vel_mps) {
    const static float s_baro_max_vel_mps = 100.;  // m/s
    const static float s_baro_max_alt_m = 10000.;  // m

    // Pulled out of my ass currently; feel free to change
    float alt_weight = 1. - expf(25. * (alt_m / s_baro_max_alt_m - 1.));
    float vel_weight_lin = 1. - vel_mps / s_baro_max_vel_mps;
    float vel_weight_exp = expf(-vel_mps / 25.);  // > 0

    if (alt_weight < 0. || vel_weight_lin < 0.) {
        return 0.;
    }

    return alt_weight * vel_weight_lin * vel_weight_exp;
}

static float se_trap_int_step(float old, float new, float dt) {
    return dt * (old + new) / 2.;
}

static float se_baro_alt_m(float p_mbar) {
    return 44330 * (1 - powf(((p_mbar) / 1013.25), 1 / 5.255f));
}

Status se_init() {
    orientation_function = get_orientation_function(DEFAULT_ORIENTATION);

    ASSERT_OK(sma_float_buffer_init(&s_baro_alt_buffer, BARO_BUFFER_SIZE),
              "failed to init baro alt buffer\n");
    ASSERT_OK(sma_float_buffer_init(&s_baro_vel_buffer, BARO_BUFFER_SIZE),
              "failed to init baro vel buffer\n");

    s_current_state_ptr = &(backup_get_ptr()->state_estimate);

    return STATUS_OK;
}

Status se_reset() {
    memset(s_current_state_ptr, 0, sizeof(StateEst));
    s_current_state_ptr->orientation.w = 1;
    return STATUS_OK;
}

Status se_set_time(float t_s) {
    s_current_state_ptr->time = t_s;
    PAL_LOGI("State estimation time reinitialized to %.1f \n", t_s);
    return STATUS_OK;
}

Status se_set_ground_pressure(float p_mbar) {
    s_ground_baro_alt_m = se_baro_alt_m(p_mbar);
    PAL_LOGI("Ground baro alt set to %.1f m\n", s_ground_baro_alt_m);
    return STATUS_OK;
}

const StateEst* se_predict() { return s_current_state_ptr; }

StateFrame se_as_frame() {
    StateFrame frame = {
        // Linear values
        .pos_vert = s_current_state_ptr->posVert,
        .vel_vert = s_current_state_ptr->velVert,
        .acc_vert = s_current_state_ptr->accVert,

        // Inertial values
        .pos_geo_x = s_current_state_ptr->posGeo.x,
        .pos_geo_y = s_current_state_ptr->posGeo.y,
        .pos_geo_z = s_current_state_ptr->posGeo.z,

        .vel_geo_x = s_current_state_ptr->velGeo.x,
        .vel_geo_y = s_current_state_ptr->velGeo.y,
        .vel_geo_z = s_current_state_ptr->velGeo.z,

        .acc_geo_x = s_current_state_ptr->accGeo.x,
        .acc_geo_y = s_current_state_ptr->accGeo.y,
        .acc_geo_z = s_current_state_ptr->accGeo.z,

        .angvel_body_x = s_current_state_ptr->angVelBody.x,
        .angvel_body_y = s_current_state_ptr->angVelBody.y,
        .angvel_body_z = s_current_state_ptr->angVelBody.z,

        // Orientation
        .orientation_w = s_current_state_ptr->orientation.w,
        .orientation_x = s_current_state_ptr->orientation.x,
        .orientation_y = s_current_state_ptr->orientation.y,
        .orientation_z = s_current_state_ptr->orientation.z,
    };

    return frame;
}

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame) {
    // Sensor timestamp is in us, so convert to seconds (float)
    float t = sensor_frame->timestamp / 1e6;
    float dt = t - s_current_state_ptr->time;
    s_current_state_ptr->time = t;

    /***********************/
    /* ACCELERATION UPDATE */
    /***********************/
    static Vector s_current_acc = {};
    Vector last_acc = s_current_acc;

    // Decide which acceleration value to use
    if (!isnan(sensor_frame->acc_i_x) &&  ///
        !isnan(sensor_frame->acc_i_y) &&  ///
        !isnan(sensor_frame->acc_i_z)) {
        // If the high-precision acceleration values are valid, prefer those
        orientation_function(sensor_frame->acc_i_x * G_MAG,  ///
                             sensor_frame->acc_i_y * G_MAG,  ///
                             sensor_frame->acc_i_z * G_MAG,  ///
                             &s_current_acc);

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
                                     &s_current_acc);
            } else {
                // If we don't, then we have to make the decision between
                // sticking with the old values or going with the new ones
                if (last_acc.x > s_current_acc.x ||  ///
                    last_acc.y > s_current_acc.y ||  ///
                    last_acc.z > s_current_acc.z) {
                    // If the old values are higher on any axis then the new
                    // values, use those since they're probably more accurate
                    s_current_acc = last_acc;
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
                             &s_current_acc);
        vec_iadd(&s_current_acc, &s_grav_vec);
    } else {
        // If neither were valid, then we reuse the acceleration values from
        // last time (which is a no-op since they're already in s_current_state)
        PAL_LOGW("Both accelerometers invalid; reusing old accelerations\n");
    }

    /******************/
    /* ANG VEL UPDATE */
    /******************/
    if (!isnan(sensor_frame->rot_i_x) &&  ///
        !isnan(sensor_frame->rot_i_y) &&  ///
        !isnan(sensor_frame->rot_i_z)) {
        orientation_function(DEG_TO_RAD(sensor_frame->rot_i_x),  ///
                             DEG_TO_RAD(sensor_frame->rot_i_y),  ///
                             DEG_TO_RAD(sensor_frame->rot_i_z),  ///
                             &(s_current_state_ptr->angVelBody));
    }

    /*******************/
    /* BARO ALT UPDATE */
    /*******************/
    static float s_current_baro_alt = 0.;
    float last_baro_alt = s_current_baro_alt;

    // Index baro alt to ground level to align with integration model
    s_current_baro_alt = sma_float_buffer_get_current_avg(&s_baro_alt_buffer) -
                         s_ground_baro_alt_m;

    // Update the baro altitude buffer if we have a valid pressure value
    if (!isnan(sensor_frame->pressure)) {
        float alt_m = se_baro_alt_m(sensor_frame->pressure);
        sma_float_buffer_insert_sample(&s_baro_alt_buffer, alt_m);
    }

    /***********************/
    /* LINEAR MODEL UPDATE */
    /***********************/
    float instant_baro_vel = (s_current_baro_alt - last_baro_alt) / dt;
    sma_float_buffer_insert_sample(&s_baro_vel_buffer, instant_baro_vel);

    float current_baro_vel =
        sma_float_buffer_get_current_avg(&s_baro_vel_buffer);

    float current_int_acc = s_current_acc.x - G_MAG;
    float current_int_vel =
        s_current_state_ptr->velVert +
        se_trap_int_step(s_current_state_ptr->accVert, current_int_acc, dt);
    float current_int_alt =
        s_current_state_ptr->posVert +
        se_trap_int_step(s_current_state_ptr->velVert, current_int_vel, dt);

    if (phase >= FP_DROGUE) {
        // After drogue deployment, use baro alt exclusively
        s_current_state_ptr->posVert = s_current_baro_alt;
        s_current_state_ptr->velVert = current_baro_vel;
        s_current_state_ptr->accVert = NAN;
    } /* else if (phase == FP_COAST_1 || phase == FP_COAST_2) {
        // During coast, use weighted combination of integration and baro
        float baro_weight = se_baro_weight(s_current_state_ptr->posVert,
                                           s_current_state_ptr->velVert);
        float int_weight = 1. - baro_weight;

        s_current_state_ptr->posVert =
            int_weight * current_int_alt + baro_weight * s_current_baro_alt;
        s_current_state_ptr->velVert =
            int_weight * current_int_vel + baro_weight * current_baro_vel;
        s_current_state_ptr->accVert = current_int_acc;
    } */
    else {
        // Otherwise, use integration exclusively
        s_current_state_ptr->posVert = current_int_alt;
        s_current_state_ptr->velVert = current_int_vel;
        s_current_state_ptr->accVert = current_int_acc;
    }

    /*************************/
    /* INERTIAL MODEL UPDATE */
    /*************************/
    Vector vec_temp;
    Quaternion quat_temp;

    Vector iacc_old = s_current_state_ptr->accGeo;
    Vector ivel_old = s_current_state_ptr->velGeo;

    quat_rot_inv(&(s_current_acc), &(s_current_state_ptr->orientation),
                 &(s_current_state_ptr->accGeo));

    vec_iadd(&(s_current_state_ptr->accGeo), &s_grav_vec);

    vec_int_step(&(s_current_state_ptr->velGeo), &iacc_old,
                 &(s_current_state_ptr->accGeo), dt, &vec_temp);
    vec_copy(&vec_temp, &(s_current_state_ptr->velGeo));

    vec_int_step(&(s_current_state_ptr->posGeo), &ivel_old,
                 &(s_current_state_ptr->posGeo), dt, &vec_temp);
    vec_copy(&vec_temp, &(s_current_state_ptr->posGeo));

    quat_step(&(s_current_state_ptr->orientation),
              &(s_current_state_ptr->angVelBody), dt, &quat_temp);
    quat_copy(&quat_temp, &(s_current_state_ptr->orientation));

    return STATUS_OK;
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
