#include "state_estimation.h"

#include <malloc.h>
#include <math.h>

#include "backup/backup.h"
#include "filter/median_filter.h"
#include "filter/sma_filter.h"
#include "quat.h"
#include "vector.h"

#define DEG_TO_RAD(x) (x * M_PI / 180)

#define CHUTE_DEPLOYED(x) \
    ((x) == FP_DROGUE || (x) == FP_MAIN || (x) == FP_LANDED)

typedef Vector* (*OrientFunc)(float, float, float, Vector*);
static OrientFunc get_orientation_function(SensorDirection up_dir);

static StateEst* s_state_ptr = NULL;

static MedianFilter s_baro_alt_median;
static SmaFilter s_baro_alt_sma;
static SmaFilter s_baro_vel_sma;

static OrientFunc orientation_function = NULL;

static Vector s_grav_vec = {.x = -G_MAG, .y = 0.0f, .z = 0.0f};

static float s_ground_alt = 0.;

enum LogState {
    SE_LOG_OK = 0,
    SE_LOG_NO_ACC = 1,
};
static LogState s_log_state = 0;

static bool se_valid_acc(float acc_g) {
    // Higher cutoff values to accomodate future sensors
    return -100. < acc_g && acc_g < 100.;
}

static bool se_valid_pressure(float pressure_mbar) {
    // Allow extended low range for descent tracking
    return 0. < pressure_mbar && pressure_mbar < 1200.;
}

static float se_baro_weight(float alt_m, float vel_mps) {
    /** WEIGHTING FUNCTIONS
     *
     * We want to trust the barometer only when we are:
     *     1. under the operational ceiling (9000 m/300 mbar)
     *     2. slow enough for averaging delay to be low
     *        and for aerodynamic effects to be negligible
     *
     * So, we have two independent weights calculated from
     * the estimated altitude and velocity each of which are
     * multiplied together (equivalent to logical AND).
     *
     * The altitude weighting function is an exponential that
     * starts off near one, then rapidly goes to zero as the
     * altitude approaches s_baro_max_alt_m. See:
     * https://www.desmos.com/calculator/wqz5wfycre
     *
     * The velocity weighting function is composed of two terms.
     * The first term is an exponential for clamping the weight to
     * 0 at s_baro_max_vel_mps. The other term is a logistic curve
     * term to smoothly transition to baro measurements. See:
     * https://www.desmos.com/calculator/da8kge4mkt
     *
     * Finally, we also only want to trust the barometer if we our averaging
     * filters are healthy, since unfiltered barometer data has too much noise.
     *
     */

    const static float s_baro_max_vel_mps = 150.;  // m/s
    const static float s_baro_max_alt_m = 9000.;   // m

    float alt_weight = 1. - expf(10. * (alt_m / s_baro_max_alt_m - 1.));

    float vel_weight_clamp = 1. - expf((vel_mps - s_baro_max_vel_mps) / 30.);
    float vel_weight_logistic =
        1. / (1. + expf((vel_mps - s_baro_max_vel_mps / 2.) / 30.));
    float vel_weight = vel_weight_clamp * vel_weight_logistic;

    // Also weight by the health of the baro filters
    float alt_median_filter_weight =
        (float)s_baro_alt_median.size / (float)s_baro_alt_median.capacity;
    float alt_sma_filter_weight =
        (float)s_baro_alt_sma.size / (float)s_baro_alt_sma.capacity;
    float alt_filter_weight = alt_median_filter_weight * alt_sma_filter_weight;
    float vel_filter_weight =
        (float)s_baro_vel_sma.size / (float)s_baro_vel_sma.capacity;
    float filter_weight = alt_filter_weight * vel_filter_weight;

    float total_weight = alt_weight * vel_weight * filter_weight;
    return total_weight < 0. ? 0. : total_weight > 1. ? 1. : total_weight;
}

static float se_trap_int_step(float old, float new, float dt) {
    return dt * (old + new) / 2.;
}

static float se_baro_alt_m(float p_mbar) {
    float alt_m = 44330 * (1 - powf(((p_mbar) / 1013.25), 1 / 5.255f));
    return se_valid_pressure(p_mbar) ? alt_m : NAN;
}

Status se_init() {
    orientation_function = get_orientation_function(DEFAULT_ORIENTATION);

    ASSERT_OK(median_filter_init(&s_baro_alt_median, BARO_ALT_MEDIAN_WINDOW),
              "failed to init baro alt median filter\n");
    ASSERT_OK(sma_filter_init(&s_baro_alt_sma, BARO_ALT_SMA_WINDOW),
              "failed to init baro alt sma filter\n");
    ASSERT_OK(sma_filter_init(&s_baro_vel_sma, BARO_VEL_SMA_WINDOW),
              "failed to init baro vel sma filter\n");

    s_state_ptr = &(backup_get_ptr()->state_estimate);

    return STATUS_OK;
}

Status se_reset() {
    sma_filter_reset(&s_baro_alt_sma);
    sma_filter_reset(&s_baro_vel_sma);
    memset(s_state_ptr, 0, sizeof(StateEst));
    s_state_ptr->orientation.w = 1;
    return STATUS_OK;
}

Status se_set_time(float t_s) {
    s_state_ptr->time = t_s;
    PAL_LOGI("State estimation time reinitialized to %.1f \n", t_s);
    return STATUS_OK;
}

Status se_set_ground_pressure(float p_mbar) {
    s_ground_alt = se_baro_alt_m(p_mbar);
    PAL_LOGI("Ground baro alt set to %.1f m\n", s_ground_alt);
    return STATUS_OK;
}

const StateEst* se_predict() { return s_state_ptr; }

StateFrame se_as_frame() {
    StateFrame frame = {
        // Linear values
        .pos_vert = s_state_ptr->posVert,
        .vel_vert = s_state_ptr->velVert,
        .acc_vert = s_state_ptr->accVert,

        // Inertial values
        .pos_geo_x = s_state_ptr->posGeo.x,
        .pos_geo_y = s_state_ptr->posGeo.y,
        .pos_geo_z = s_state_ptr->posGeo.z,

        .vel_geo_x = s_state_ptr->velGeo.x,
        .vel_geo_y = s_state_ptr->velGeo.y,
        .vel_geo_z = s_state_ptr->velGeo.z,

        .acc_geo_x = s_state_ptr->accGeo.x,
        .acc_geo_y = s_state_ptr->accGeo.y,
        .acc_geo_z = s_state_ptr->accGeo.z,

        .angvel_body_x = s_state_ptr->angVelBody.x,
        .angvel_body_y = s_state_ptr->angVelBody.y,
        .angvel_body_z = s_state_ptr->angVelBody.z,

        // Orientation
        .orientation_w = s_state_ptr->orientation.w,
        .orientation_x = s_state_ptr->orientation.x,
        .orientation_y = s_state_ptr->orientation.y,
        .orientation_z = s_state_ptr->orientation.z,
    };

    return frame;
}

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame) {
    // Sensor timestamp is in us, so convert to seconds (float)
    float t = sensor_frame->timestamp / 1e6;
    float dt = t - s_state_ptr->time;
    s_state_ptr->time = t;

    /***********************/
    /* ACCELERATION UPDATE */
    /***********************/
    static Vector s_current_acc = {};
    Vector last_acc = s_current_acc;

    // Decide which acceleration value to use
    if (se_valid_acc(sensor_frame->acc_i_x) &&  ///
        se_valid_acc(sensor_frame->acc_i_y) &&  ///
        se_valid_acc(sensor_frame->acc_i_z)) {
        s_log_state = SE_LOG_OK;

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
            if (se_valid_acc(sensor_frame->acc_h_x) &&
                se_valid_acc(sensor_frame->acc_h_y) &&
                se_valid_acc(sensor_frame->acc_h_z)) {
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
                    // If the old values are higher on any axis than the new
                    // values, use those since they're probably more accurate
                    s_current_acc = last_acc;
                }
            }
        }
    } else if (se_valid_acc(sensor_frame->acc_h_x) &&  ///
               se_valid_acc(sensor_frame->acc_h_y) &&  ///
               se_valid_acc(sensor_frame->acc_h_z)) {
        s_log_state = SE_LOG_OK;
        // If the high-precision values are invalid but the high-range ones are
        // valid, not much to think about -- just go with the high-range ones
        orientation_function(sensor_frame->acc_h_x * G_MAG,  ///
                             sensor_frame->acc_h_y * G_MAG,  ///
                             sensor_frame->acc_h_z * G_MAG,  ///
                             &s_current_acc);
    } else {
        // If neither were valid, then we reuse the acceleration values from
        // last time (which is a no-op since they're in s_current_state).
        PAL_STATE_LOGW(
            &s_log_state, SE_LOG_NO_ACC,
            "Both accelerometers invalid; reusing old accelerations\n");

        // If we have a complete failure of both accelerometers, this will
        // repeat many, many times. If that happens, we want the stored
        // acceleration value to decay so that the barometer is given more
        // confidence or so that we eventually deploy from the gravity offset.
        vec_iscale(&s_current_acc, 0.9);  // half-life of 7 iterations
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
                             &(s_state_ptr->angVelBody));
    }

    /*******************/
    /* BARO ALT UPDATE */
    /*******************/
    float baro_alt = se_baro_alt_m(sensor_frame->pressure) - s_ground_alt;

    /***********************/
    /* LINEAR MODEL UPDATE */
    /***********************/
    // Baro updates
    float last_baro_alt = s_state_ptr->posBaro;

    // Median filter on the first stage for outlier rejection
    median_filter_insert(&s_baro_alt_median, baro_alt);
    float median_baro_alt = median_filter_get_median(&s_baro_alt_median);

    // SMA filter on the second stage for smoothing
    sma_filter_insert(&s_baro_alt_sma, median_baro_alt);
    s_state_ptr->posBaro = sma_filter_get_mean(&s_baro_alt_sma);

    // Calculate baro velocity using last and current average baro altitude
    float baro_vel = (s_state_ptr->posBaro - last_baro_alt) / dt;

    // SMA filter on the velocity calculated above
    sma_filter_insert(&s_baro_vel_sma, baro_vel);
    s_state_ptr->velBaro = sma_filter_get_mean(&s_baro_vel_sma);

    // If the filters empty out, the baro estimates can be NAN, so make sure to
    // not infect the main state estimates with the NANovirus
    bool baro_valid =
        !isnan(s_state_ptr->posBaro) && !isnan(s_state_ptr->velBaro);

    // Acceleration updates
    float last_imu_acc = s_state_ptr->accImu;
    float last_imu_vel = s_state_ptr->velImu;

    // Trapezoidal integration to update IMU estimates
    s_state_ptr->accImu = s_current_acc.x - G_MAG;
    s_state_ptr->velImu +=
        se_trap_int_step(s_state_ptr->accImu, last_imu_acc, dt);
    s_state_ptr->posImu +=
        se_trap_int_step(s_state_ptr->velImu, last_imu_vel, dt);

    // Combined state updates
    if (phase >= FP_DROGUE) {
        // After drogue deployment, use baro alt exclusively
        s_state_ptr->posVert = s_state_ptr->posBaro;
        s_state_ptr->velVert = s_state_ptr->velBaro;
        s_state_ptr->accVert = vec_mag(&s_current_acc);
    } else if (phase == FP_COAST && baro_valid) {
        // During coast, use weighted combination of integration and baro
        static float baro_weight = 0;
        float new_baro_weight =
            se_baro_weight(s_state_ptr->posVert, s_state_ptr->velVert);
        baro_weight = (baro_weight + new_baro_weight) / 2;
        // baro_weight = 0.;
        float imu_weight = 1. - baro_weight;

        s_state_ptr->posVert = imu_weight * s_state_ptr->posImu +
                               baro_weight * s_state_ptr->posBaro;
        s_state_ptr->velVert = imu_weight * s_state_ptr->velImu +
                               baro_weight * s_state_ptr->velBaro;
        s_state_ptr->accVert = s_state_ptr->accImu;
    } else {
        // Otherwise, use integration exclusively
        s_state_ptr->posVert = s_state_ptr->posImu;
        s_state_ptr->velVert = s_state_ptr->velImu;
        s_state_ptr->accVert = s_state_ptr->accImu;
    }

    /*************************/
    /* INERTIAL MODEL UPDATE */
    /*************************/
    Vector vec_temp;
    Quaternion quat_temp;

    Vector iacc_old = s_state_ptr->accGeo;
    Vector ivel_old = s_state_ptr->velGeo;

    quat_rot_inv(&(s_current_acc), &(s_state_ptr->orientation),
                 &(s_state_ptr->accGeo));

    vec_iadd(&(s_state_ptr->accGeo), &s_grav_vec);

    vec_int_step(&(s_state_ptr->velGeo), &iacc_old, &(s_state_ptr->accGeo), dt,
                 &vec_temp);
    vec_copy(&vec_temp, &(s_state_ptr->velGeo));

    vec_int_step(&(s_state_ptr->posGeo), &ivel_old, &(s_state_ptr->posGeo), dt,
                 &vec_temp);
    vec_copy(&vec_temp, &(s_state_ptr->posGeo));

    quat_step(&(s_state_ptr->orientation), &(s_state_ptr->angVelBody), dt,
              &quat_temp);
    quat_copy(&quat_temp, &(s_state_ptr->orientation));

    return STATUS_OK;
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
