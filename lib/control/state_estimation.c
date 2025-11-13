#include "state_estimation.h"

#include <math.h>
#include <string.h>

#include "backup/backup.h"
#include "filter/median_filter.h"
#include "filter/sma_filter.h"
#include "kalman.h"
#include "quat.h"
#include "sample_window.h"
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

static SampleWindow s_baro_alt_window;
static SampleWindow s_baro_time_window;

static OrientFunc orientation_function = NULL;

static Vector s_grav_vec = {.x = -G_MAG, .y = 0.f, .z = 0.f};

static float* s_ground_alt_ptr;

enum LogState {
    SE_LOG_OK = 0,
    SE_LOG_NO_ACC = 1,
};
static LogState s_log_state = 0;

static bool se_valid_acc(float acc_g) {
    // Higher cutoff values to accomodate future sensors
    return -100.f < acc_g && acc_g < 100.f;
}

static bool se_valid_pressure(float pressure_mbar) {
    // Allow extended low range for descent tracking
    return 0.f < pressure_mbar && pressure_mbar < 1100.f;
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

    const static float s_baro_max_vel_mps = 90.f;  // m/s
    const static float s_baro_max_alt_m = 9000.f;  // m

    float alt_weight = 1.f - expf(10. * (alt_m / s_baro_max_alt_m - 1.f));

    float vel_weight_clamp = 1.f - expf((vel_mps - s_baro_max_vel_mps) / 9.f);
    float vel_weight_logistic =
        1. / (1. + expf((vel_mps - s_baro_max_vel_mps / 2.) / 9.));
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
    return total_weight < 0.f ? 0.f : total_weight > 1.f ? 1.f : total_weight;
}

static float se_trap_int_step(float old, float new, float dt) {
    return dt * (old + new) / 2.f;
}

static float se_baro_alt_m(float p_mbar) {
    float alt_m = 44330.f * (1.f - powf(((p_mbar) / 1013.25f), 1.f / 5.255f));
    return se_valid_pressure(p_mbar) ? alt_m : NAN;
}

Status se_init() {
    ASSERT_OK(median_filter_init(&s_baro_alt_median, BARO_ALT_MEDIAN_WINDOW),
              "failed to init baro alt median filter\n");
    ASSERT_OK(sma_filter_init(&s_baro_alt_sma, BARO_ALT_SMA_WINDOW),
              "failed to init baro alt sma filter\n");
    ASSERT_OK(sma_filter_init(&s_baro_vel_sma, BARO_VEL_SMA_WINDOW),
              "failed to init baro vel sma filter\n");
    ASSERT_OK(sample_window_init(&s_baro_alt_window, BARO_DIFF_WINDOW),
              "failed to init baro diff alt window\n");
    ASSERT_OK(sample_window_init(&s_baro_time_window, BARO_DIFF_WINDOW),
              "failed to init baro diff time window\n");

    s_state_ptr = &(backup_get_ptr()->state_estimate);
    s_ground_alt_ptr = &(backup_get_ptr()->ground_alt_m);

    SensorDirection sensor_dir =
        config_get_ptr()->orient_antenna_up ? IMU_Z_DOWN : IMU_Z_UP;

    orientation_function = get_orientation_function(sensor_dir);

    // kf init
    ASSERT_OK(kf_init_mats(), "failed to allocate memory for kf matrices");
    mfloat x0[NUM_TOT_STATES] = {0, 0, 0, 1,
                                 0, 0, 0};  // TODO: get these from config file
    mfloat P0_diag[NUM_TOT_STATES] = {10, 5, 1, 1, 1, 1, 1};

    // kf direction enum defined differently than this one
    int kf_up = sensor_dir + 1;
    if (kf_up > 3) {
        kf_up = -kf_up + 3;
    }

    kf_init_state(x0, P0_diag);

    return STATUS_OK;
}

Status se_reset() {
    median_filter_reset(&s_baro_alt_median);
    sma_filter_reset(&s_baro_alt_sma);
    sma_filter_reset(&s_baro_vel_sma);
    sample_window_reset(&s_baro_alt_window);
    sample_window_reset(&s_baro_time_window);
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
    *s_ground_alt_ptr = se_baro_alt_m(p_mbar);
    kf_set_initial_alt(*s_ground_alt_ptr);
    PAL_LOGI("Ground baro alt set to %.1f m\n", *s_ground_alt_ptr);
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

        .orient_geo_w = s_state_ptr->orientation.w,
        .orient_geo_x = s_state_ptr->orientation.x,
        .orient_geo_y = s_state_ptr->orientation.y,
        .orient_geo_z = s_state_ptr->orientation.z,

        // ekf values
        .pos_ekf = s_state_ptr->posEkf,
        .vel_ekf = s_state_ptr->velEkf,
        .acc_ekf = s_state_ptr->accEkf,

        .orient_ekf_1 = s_state_ptr->orientEkf1,
        .orient_ekf_2 = s_state_ptr->orientEkf2,
        .orient_ekf_3 = s_state_ptr->orientEkf3,
        .orient_ekf_4 = s_state_ptr->orientEkf4,

        .pos_var_ekf = s_state_ptr->posVarEkf,
        .vel_var_ekf = s_state_ptr->velVarEkf,
        .acc_var_ekf = s_state_ptr->accVarEkf,

        .orient_var_ekf_1 = s_state_ptr->orientVarEkf1,
        .orient_var_ekf_2 = s_state_ptr->orientVarEkf2,
        .orient_var_ekf_3 = s_state_ptr->orientVarEkf3,
        .orient_var_ekf_4 = s_state_ptr->orientVarEkf4,
        // We could change these to w x y z but I tried to change it and it
        // broke
    };

    return frame;
}

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame) {
    // Sensor timestamp is in us, so convert to seconds (float)
    float t = sensor_frame->timestamp / 1e6f;
    float dt = t - s_state_ptr->time;
    s_state_ptr->time = t;

    /***********************/
    /* ACCELERATION UPDATE */
    /***********************/
    static Vector s_current_acc;

    Vector acc_h;
    Vector acc_i;

    orientation_function(sensor_frame->acc_h_x,  ///
                         sensor_frame->acc_h_y,  ///
                         sensor_frame->acc_h_z,  ///
                         &acc_h);
    orientation_function(sensor_frame->acc_i_x,  ///
                         sensor_frame->acc_i_y,  ///
                         sensor_frame->acc_i_z,  ///
                         &acc_i);

    // Decide which acceleration value to use
    if (se_valid_acc(acc_i.x) &&  ///
        se_valid_acc(acc_i.y) &&  ///
        se_valid_acc(acc_i.z)) {
        s_log_state = SE_LOG_OK;

        // If the high-precision acceleration values are valid, prefer those
        vec_scale(&acc_i, G_MAG, &s_current_acc);

        // Check if we're exceeding the high-precision limit on any axis
        if (fabsf(acc_i.x) > LOW_G_MAX_ACC ||  ///
            fabsf(acc_i.y) > LOW_G_MAX_ACC ||  ///
            fabsf(acc_i.z) > LOW_G_MAX_ACC) {
            // If we are, check if we have valid high-range values
            if (se_valid_acc(acc_h.x) &&  ///
                se_valid_acc(acc_h.y) &&  ///
                se_valid_acc(acc_h.z)) {
                // If we do, then, just use those straight up
                vec_scale(&acc_h, G_MAG, &s_current_acc);
            }
        }
    } else if (se_valid_acc(acc_h.x) &&  ///
               se_valid_acc(acc_h.y) &&  ///
               se_valid_acc(acc_h.z)) {
        s_log_state = SE_LOG_OK;
        // If the high-precision values are invalid but the high-range ones are
        // valid, not much to think about -- just go with the high-range ones
        vec_scale(&acc_h, G_MAG, &s_current_acc);
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
    Vector rot;

    orientation_function(DEG_TO_RAD(sensor_frame->rot_i_x),  ///
                         DEG_TO_RAD(sensor_frame->rot_i_y),  ///
                         DEG_TO_RAD(sensor_frame->rot_i_z),  ///
                         &rot);

    if (!isnan(rot.x) &&  ///
        !isnan(rot.y) &&  ///
        !isnan(rot.z)) {
        s_state_ptr->angVelBody = rot;
    }

    /*******************/
    /* BARO ALT UPDATE */
    /*******************/
    float pressure = sensor_frame->pressure;
    float baro_alt = se_baro_alt_m(pressure) - *s_ground_alt_ptr;

    /***********************/
    /* LINEAR MODEL UPDATE */
    /***********************/
    // Median filter on the first stage for outlier rejection
    median_filter_insert(&s_baro_alt_median, baro_alt);
    float median_baro_alt = median_filter_get_median(&s_baro_alt_median);

    // SMA filter on the second stage for smoothing
    sma_filter_insert(&s_baro_alt_sma, median_baro_alt);
    s_state_ptr->posBaro = sma_filter_get_mean(&s_baro_alt_sma);

    // Calculate baro velocity using the differentiaton window
    sample_window_insert(&s_baro_alt_window, s_state_ptr->posBaro);
    sample_window_insert(&s_baro_time_window, s_state_ptr->time);
    float old_baro_alt = sample_window_get(&s_baro_alt_window, 0);
    float old_baro_time = sample_window_get(&s_baro_time_window, 0);
    float baro_alt_diff = s_state_ptr->posBaro - old_baro_alt;
    float baro_time_diff = s_state_ptr->time - old_baro_time;
    float baro_vel = baro_alt_diff / baro_time_diff;

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

    // Always update acceleration
    s_state_ptr->accVert = s_state_ptr->accImu;

    // Combined state updates
    if (FP_BOOST <= phase && phase <= FP_MAIN) {
        if (phase == FP_DROGUE || phase == FP_MAIN) {
            // During descent, use baro alt exclusively
            s_state_ptr->posVert = s_state_ptr->posBaro;
            s_state_ptr->velVert = s_state_ptr->velBaro;
        } else if (phase == FP_COAST && baro_valid) {
            // During coast, use weighted combination of integration and baro
            static float s_baro_weight = 0.f;
            float new_baro_weight =
                se_baro_weight(s_state_ptr->posVert, s_state_ptr->velVert);
            s_baro_weight = (s_baro_weight + new_baro_weight) / 2.f;
            float imu_weight = 1.f - s_baro_weight;

            s_state_ptr->posVert = imu_weight * s_state_ptr->posImu +
                                   s_baro_weight * s_state_ptr->posBaro;
            s_state_ptr->velVert = imu_weight * s_state_ptr->velImu +
                                   s_baro_weight * s_state_ptr->velBaro;
        } else {
            // In all other cases, use integration exclusively
            s_state_ptr->posVert = s_state_ptr->posImu;
            s_state_ptr->velVert = s_state_ptr->velImu;
        }
    } else {
        // If we're on the ground, skip all state updates
        return STATUS_OK;
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

    vec_int_step(&(s_state_ptr->posGeo), &ivel_old, &(s_state_ptr->velGeo), dt,
                 &vec_temp);
    vec_copy(&vec_temp, &(s_state_ptr->posGeo));

    quat_step(&(s_state_ptr->orientation), &(s_state_ptr->angVelBody), dt,
              &quat_temp);
    quat_copy(&quat_temp, &(s_state_ptr->orientation));

    /********************/
    /* EKF MODEL UPDATE */
    /********************/
    KfInputVector kf_input = {
        .pressure = se_valid_pressure(pressure) ? pressure : NAN,
        .acc_h = se_valid_acc(acc_h.x) ? acc_h.x : NAN,
        .acc_i = se_valid_acc(acc_i.x) ? acc_i.x : NAN,
        .rot_x = rot.x,
        .rot_y = rot.y,
        .rot_z = rot.z,
    };

    kf_do_kf(phase, kf_input, dt);  // TODO: status output here
    kf_write_state(s_state_ptr);    // write new state to StateEst

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
