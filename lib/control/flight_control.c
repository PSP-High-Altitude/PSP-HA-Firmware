#include "flight_control.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "backup/backup.h"
#include "board_config.h"
#include "cond_timer.h"
#include "pyros.h"
#include "state_estimation.h"
#include "timer.h"

// Forward declarations
FlightPhase fp_update_init(const SensorFrame* sensor_frame);
FlightPhase fp_update_wait(const SensorFrame* sensor_frame);
FlightPhase fp_update_ready(const SensorFrame* sensor_frame);
FlightPhase fp_update_boost(const SensorFrame* sensor_frame);
FlightPhase fp_update_coast(const SensorFrame* sensor_frame);
FlightPhase fp_update_drogue(const SensorFrame* sensor_frame);
FlightPhase fp_update_main(const SensorFrame* sensor_frame);
FlightPhase fp_update_landed(const SensorFrame* sensor_frame);
FlightPhase fp_update_error(const SensorFrame* sensor_frame);

FlightStageStatus fp_stage_check_drogue(const StateEst* state);
FlightStageStatus fp_stage_check_sep_lockout(const StateEst* state);
FlightStageStatus fp_stage_check_ignite_lockout(const StateEst* state);

// Pointer to flight phase
static FlightPhase* s_flight_phase_ptr = NULL;

// Pointer to config object
static BoardConfig* s_config_ptr = NULL;

// Initialization start time
static uint64_t s_init_start_ms;

// Launch detect sensor data buffer
static SensorFrame* s_ld_buffer_data;
static size_t s_ld_buffer_size = 0;

// Condition timers
static CondTimer s_init_timer;
static CondTimer s_boost_det_timer;
static CondTimer s_landing_det_timer;
static CondTimer s_drogue_delay_timer;
static CondTimer s_stage_sep_timer;

static uint64_t s_launch_time_ms = 0;
static uint64_t s_boost_time_ms = 0;
static uint64_t s_apogee_time_ms = 0;
static bool s_stage_sep_locked = false;
static bool s_stage_ignite_locked = false;

static uint64_t ms_since_launch() {
    if (s_launch_time_ms) {
        return MILLIS() - s_launch_time_ms;
    }

    return 0;
}

static uint64_t ms_since_boost() {
    if (s_boost_time_ms) {
        return MILLIS() - s_boost_time_ms;
    }

    return 0;
}

Status fp_init() {
    s_init_start_ms = MILLIS();

    s_config_ptr = config_get_ptr();
    if (s_config_ptr == NULL) {
        ASSERT_OK(STATUS_STATE_ERROR, "unable to get ptr to config\n");
    }

    s_flight_phase_ptr = &(backup_get_ptr()->flight_phase);
    if (s_flight_phase_ptr == NULL) {
        ASSERT_OK(STATUS_STATE_ERROR, "unable to get ptr to flight phase\n");
    }

    if (s_config_ptr->launch_detect_replay) {
        // Allocate buffer for storing sensor data during launch detection
        // Dynamic allocation is gross, but it's during init so should be safe
        s_ld_buffer_size = 1 + (s_config_ptr->boost_detect_period_ms /
                                s_config_ptr->control_loop_period_ms);

        s_ld_buffer_data = malloc(sizeof(SensorFrame) * s_ld_buffer_size);

        if (s_ld_buffer_data == NULL) {
            ASSERT_OK(STATUS_MEMORY_ERROR,
                      "failed to allocate launch replay buffer");
        }

        PAL_LOGI("Allocated %u entry buffer for launch replay\n",
                 s_ld_buffer_size);
    }

    // Initialize condition timers
    cond_timer_init(&s_init_timer, s_config_ptr->state_init_time_ms);
    cond_timer_init(&s_boost_det_timer, s_config_ptr->boost_detect_period_ms);
    cond_timer_init(&s_landing_det_timer, s_config_ptr->min_grounded_time_ms);
    cond_timer_init(&s_drogue_delay_timer, s_config_ptr->drogue_delay_ms);
    cond_timer_init(&s_stage_sep_timer, s_config_ptr->stage_sep_delay_ms);

#ifdef HWIL_TEST
    // Always start from init for HWIL test
    *s_flight_phase_ptr = FP_INIT;
#endif

    if (*s_flight_phase_ptr <= FP_READY || *s_flight_phase_ptr >= FP_LANDED) {
        PAL_LOGI("Resetting flight logic and state estimation\n");
        ASSERT_OK(se_reset(), "failed to reset state\n");
        *s_flight_phase_ptr = FP_INIT;
    }

    return STATUS_OK;
}

FlightPhase fp_get() { return *s_flight_phase_ptr; }

Status fp_update(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state estimation update failed\n");

    switch (*s_flight_phase_ptr) {
        case FP_INIT:
            *s_flight_phase_ptr = fp_update_init(sensor_frame);
            break;

        case FP_WAIT:
            *s_flight_phase_ptr = fp_update_wait(sensor_frame);
            break;

        case FP_READY:
            *s_flight_phase_ptr = fp_update_ready(sensor_frame);
            break;

        case FP_BOOST:
            *s_flight_phase_ptr = fp_update_boost(sensor_frame);
            break;

        case FP_COAST:
            *s_flight_phase_ptr = fp_update_coast(sensor_frame);
            break;

        case FP_DROGUE:
            *s_flight_phase_ptr = fp_update_drogue(sensor_frame);
            break;

        case FP_MAIN:
            *s_flight_phase_ptr = fp_update_main(sensor_frame);
            break;

        case FP_LANDED:
            *s_flight_phase_ptr = fp_update_landed(sensor_frame);
            break;

        case FP_ERROR:
            *s_flight_phase_ptr = fp_update_error(sensor_frame);
            break;

        default:
            // Unknown state; go back to init because there's not really
            // much else that we can realistically do at this point
            *s_flight_phase_ptr = fp_update_init(sensor_frame);
            break;
    }

    return STATUS_OK;
}

FlightPhase fp_update_init(const SensorFrame* sensor_frame) {
    // During init, we want to record a reliable pressure value we can use to
    // determine the ground altitude later on, and check that the accelerometers
    // are outputting values near 1g to prevent launch detection mishaps.
    static float s_pressure_sum = 0;
    static size_t s_pressure_num_pts = 0;

    static float s_acc_h_sum = 0;
    static size_t s_acc_h_num_pts = 0;

    static float s_acc_i_sum = 0;
    static size_t s_acc_i_num_pts = 0;

    float acc_h_mag = sqrtf(sensor_frame->acc_h_x * sensor_frame->acc_h_x +
                            sensor_frame->acc_h_y * sensor_frame->acc_h_y +
                            sensor_frame->acc_h_z * sensor_frame->acc_h_z);

    float acc_i_mag = sqrtf(sensor_frame->acc_i_x * sensor_frame->acc_i_x +
                            sensor_frame->acc_i_y * sensor_frame->acc_i_y +
                            sensor_frame->acc_i_z * sensor_frame->acc_i_z);

    if (!isnan(sensor_frame->pressure)) {
        s_pressure_sum += sensor_frame->pressure;
        s_pressure_num_pts += 1;
    }

    if (!isnan(acc_h_mag)) {
        s_acc_h_sum += acc_h_mag;
        s_acc_h_num_pts += 1;
    }

    if (!isnan(acc_i_mag)) {
        s_acc_i_sum += acc_i_mag;
        s_acc_i_num_pts += 1;
    }

    if (cond_timer_update(&s_init_timer, true)) {
        // Once the init time is up, record the average ground pressure
        se_set_ground_pressure(s_pressure_sum / s_pressure_num_pts);

        float acc_h_avg_mag = s_acc_h_sum / s_acc_h_num_pts;
        float acc_i_avg_mag = s_acc_i_sum / s_acc_i_num_pts;

        // If both accelerometers are invalid at this point, abort
        if (isnan(acc_h_mag) && isnan(acc_i_mag)) {
            PAL_LOGE("Both accelerometers invalid; aborting\n");
            PAL_LOGI("FP_INIT -> FP_ERROR");
            return FP_ERROR;
        }

        // Check that high-g accelerometer was within range
        if (fabsf(acc_h_avg_mag - 1) * G_MAG >
            s_config_ptr->max_ready_acc_bias_mps2) {
            PAL_LOGE(
                "High-g accelerometer avg %.3f is unacceptable; aborting\n",
                acc_h_avg_mag);
            PAL_LOGI("FP_INIT -> FP_ERROR");
            return FP_ERROR;
        }

        // Check that low-g accelerometer was within range
        if (fabsf(acc_i_avg_mag - 1) * G_MAG >
            s_config_ptr->max_ready_acc_bias_mps2) {
            PAL_LOGE("Low-g accelerometer avg %.3f is unacceptable; aborting\n",
                     acc_i_avg_mag);
            PAL_LOGI("FP_INIT -> FP_ERROR");
            return FP_ERROR;
        }

        PAL_LOGI("FP_INIT -> FP_WAIT\n");
        return FP_WAIT;
    }

    // Otherwise, stay in init
    return FP_INIT;
}

FlightPhase fp_update_wait(const SensorFrame* sensor_frame) {
    // If the acceleration is significantly negative, then that implies that we
    // are not correctly positioned for launch. When the rocket is nose up on
    // the pad, we should measure a near-zero vertical acceleration (accounting
    // for some accelerometer bias). If we're measuring a significantly negative
    // acceleration, that implies that the rocket is not vertical (at least in
    // the frame of the board). This should prevent launch detection from
    // running before it's vertical on the pad, and should alert us to potential
    // misconfiguration of the board axes if we see this state on the pad;
    // e.g. https://www.npr.org/sections/thetwo-way/2013/07/10/200775748.
    const StateEst* state = se_predict();

    if (state->accVert > -s_config_ptr->max_ready_acc_bias_mps2) {
        PAL_LOGI("FP_WAIT -> FP_READY\n");
        return FP_READY;
    }

    return FP_WAIT;
}

FlightPhase fp_update_ready(const SensorFrame* sensor_frame) {
    // In ready, we're tracking the acceleration for launch
    // detection and buffering data so that we can replay
    // it through the state estimation when launch is detected
    static size_t s_ld_buffer_entries = 0;

    const StateEst* state = se_predict();

    if (state->accVert < -s_config_ptr->max_ready_acc_bias_mps2) {
        // Go to the wait state if the acceleration is highly negative
        PAL_LOGI("FP_READY -> FP_WAIT\n");
        return FP_WAIT;
    }

    bool launch_detected = false;
    if (!isnan(state->accVert)) {
        // Only include non-NAN values in launch detection decisions
        bool acc_above_threshold =
            state->accVert > s_config_ptr->min_boost_acc_mps2;

        launch_detected =
            cond_timer_update(&s_boost_det_timer, acc_above_threshold);

        if (acc_above_threshold && s_config_ptr->launch_detect_replay) {
            // If we need to replay launch, save the current frame in the buffer
            if (s_ld_buffer_entries < s_ld_buffer_size) {
                s_ld_buffer_data[s_ld_buffer_entries++] = *sensor_frame;
            }
        } else {
            // If we're below the threshold, reset the buffer
            s_ld_buffer_entries = 0;
        }
    }

    if (launch_detected) {
        s_launch_time_ms = MILLIS() - s_config_ptr->boost_detect_period_ms;

        if (s_config_ptr->launch_detect_replay) {
            // Reset the state estimation since we're
            // replaying from the new start of the flight
            se_reset();

            // Resetting will clear the state estimation time, so we deal
            // with that by setting it to the time of the first stored
            // sensor frame minus the control loop period
            se_set_time(s_ld_buffer_data[0].timestamp / 1e6 -
                        s_config_ptr->control_loop_period_ms / 1e3);

            for (int i = 0; i < s_ld_buffer_entries; i++) {
                EXPECT_OK(se_update(*s_flight_phase_ptr, &s_ld_buffer_data[i]),
                          "state est update failed during launch replay\n");
            }
        }

        // Reset the boost detection timer for use in boost
        cond_timer_update(&s_boost_det_timer, false);

        PAL_LOGI("FP_READY -> FP_BOOST\n");
        return FP_BOOST;
    }

    return FP_READY;
}

FlightPhase fp_update_boost(const SensorFrame* sensor_frame) {
    const StateEst* state = se_predict();

    if (state->accVert < s_config_ptr->max_coast_acc_mps2) {
        // If we're above the coast deceleration threshold, transition.
        cond_timer_update(&s_boost_det_timer, false);
        s_boost_time_ms = MILLIS();
        PAL_LOGI("FP_BOOST -> FP_COAST\n");
        return FP_COAST;
    }

    bool acc_below_threshold =
        state->accVert < s_config_ptr->min_boost_acc_mps2;

    if (cond_timer_update(&s_boost_det_timer, acc_below_threshold)) {
        // If we were below the boost threshold for the boost detection period,
        // also transition to coast. This is needed to make sure that we go to
        // coast in low deceleration scenarios or sensor malfunctions.
        cond_timer_update(&s_boost_det_timer, false);
        s_boost_time_ms = MILLIS();
        PAL_LOGI("FP_BOOST -> FP_COAST\n");
        return FP_COAST;
    }

    return FP_BOOST;
}

FlightPhase fp_update_coast(const SensorFrame* sensor_frame) {
    const StateEst* state = se_predict();

    // Check if we've reached apogee BEFORE ANYTHING ELSE
    FlightStageStatus drogue_status = fp_stage_check_drogue(state);

    if (drogue_status == FP_STG_GO) {
        // Fire the drogue
        Status fired =
            EXPECT_OK(pyros_fire(PYRO_DRG), "failed to fire drogue pyro\n");

        // Only transition to drogue if we successfully fired
        if (fired == STATUS_OK) {
            PAL_LOGI("FP_COAST -> FP_DROGUE\n");
            return FP_DROGUE;
        }
    }

    // Check if we should separate
    FlightStageStatus sep_status = fp_stage_check_sep_lockout(state);
    if (sep_status == FP_STG_GO) {
        // Separate if GO
        EXPECT_OK(pyros_fire(s_config_ptr->stage_sep_pyro_channel),
                  "failed to fire stage sep pyro\n");
    } else if (sep_status == FP_STG_WAIT) {
        // If we're waiting, don't check further conditions
        return FP_COAST;
    }

    // Check if we should ignite
    FlightStageStatus ignite_status = fp_stage_check_ignite_lockout(state);

    if (ignite_status == FP_STG_GO) {
        // Ignite if GO
        EXPECT_OK(pyros_fire(s_config_ptr->stage_ignite_pyro_channel),
                  "failed to fire motor ignitor\n");
    } else if (ignite_status == FP_STG_WAIT) {
        // If we're waiting, don't check further conditions
        return FP_COAST;
    }

    // Check if we should go to boost
    bool acc_above_threshold =
        state->accVert > s_config_ptr->min_boost_acc_mps2;

    if (cond_timer_update(&s_boost_det_timer, acc_above_threshold)) {
        // We were above the boost threshold for the detection period
        cond_timer_update(&s_boost_det_timer, false);
        PAL_LOGI("FP_COAST -> FP_BOOST\n");
        return FP_BOOST;
    }

    return FP_COAST;
}

FlightPhase fp_update_drogue(const SensorFrame* sensor_frame) {
    const StateEst* state = se_predict();

    if (state->posVert < s_config_ptr->main_height_m) {
        EXPECT_OK(pyros_fire(PYRO_MAIN), "failed to fire main pyro\n");

        PAL_LOGI("FP_DROGUE -> FP_MAIN\n");
        return FP_MAIN;
    }

    return FP_DROGUE;
}

FlightPhase fp_update_main(const SensorFrame* sensor_frame) {
    const StateEst* state = se_predict();

    bool alt_below_threshold =
        state->posVert < s_config_ptr->max_grounded_alt_m;

    if (cond_timer_update(&s_landing_det_timer, alt_below_threshold)) {
        PAL_LOGI("FP_MAIN -> FP_LANDED\n");
        return FP_LANDED;
    }

    return FP_MAIN;
}

FlightPhase fp_update_landed(const SensorFrame* sensor_frame) {
    // Nothing more to be done
    return FP_LANDED;
}

FlightPhase fp_update_error(const SensorFrame* sensor_frame) {
    // Nothing more to be done
    return FP_ERROR;
}

FlightStageStatus fp_stage_check_drogue(const StateEst* state) {
    // If we're below the deployment lockout threshold, return immediately
    if (ms_since_launch() < s_config_ptr->deploy_lockout_ms) {
        return FP_STG_WAIT;
    }

    // Use a threshold slightly above zero for deployment to account for cases
    // where sensor malfunction causes  velocity to get stuck at/near zero.
    if (state->velVert < 1) {
        // Record the apogee time
        if (!s_apogee_time_ms) {
            s_apogee_time_ms = MILLIS();
        }

        if (cond_timer_update(&s_drogue_delay_timer, true)) {
            return FP_STG_GO;
        }
    }

    return FP_STG_WAIT;
}

FlightStageStatus fp_stage_check_sep_lockout(const StateEst* state) {
    if (s_stage_sep_locked) {
        return FP_STG_NOGO;
    }

    if (!s_config_ptr->stage_is_separator_bool) {
        s_stage_sep_locked = 1;
        return FP_STG_NOGO;
    }

    if (ms_since_launch() < s_config_ptr->stage_sep_lockout_ms) {
        return FP_STG_WAIT;
    }

    if (ms_since_boost() < s_config_ptr->stage_sep_delay_ms) {
        return FP_STG_WAIT;
    }

    s_stage_sep_locked = 1;

    float velocity = state->velVert;
    float altitude = state->posVert;
    float angle_deg =
        quat_angle_from_vertical(&(state->orientation)) * M_PI / 180.0f;

    if (velocity < s_config_ptr->stage_min_sep_velocity_mps ||
        velocity > s_config_ptr->stage_max_sep_velocity_mps) {
        return FP_STG_NOGO;
    }

    if (altitude < s_config_ptr->stage_min_sep_altitude_m ||
        altitude > s_config_ptr->stage_max_sep_altitude_m) {
        return FP_STG_NOGO;
    }

    if (angle_deg < s_config_ptr->stage_min_sep_angle_deg ||
        angle_deg > s_config_ptr->stage_max_sep_angle_deg) {
        return FP_STG_NOGO;
    }

    return FP_STG_GO;
}

FlightStageStatus fp_stage_check_ignite_lockout(const StateEst* state) {
    if (s_stage_ignite_locked) {
        return FP_STG_NOGO;
    }

    if (!s_config_ptr->stage_is_igniter_bool) {
        s_stage_ignite_locked = 1;
        return FP_STG_NOGO;
    }

    if (ms_since_launch() < s_config_ptr->stage_ignite_lockout_ms) {
        return FP_STG_WAIT;
    }

    s_stage_ignite_locked = 1;

    float velocity = state->velVert;
    float altitude = state->posVert;
    float angle_deg =
        quat_angle_from_vertical(&(state->orientation)) * M_PI / 180.0f;

    if (velocity < s_config_ptr->stage_min_ignite_velocity_mps ||
        velocity > s_config_ptr->stage_max_ignite_velocity_mps) {
        return FP_STG_NOGO;
    }

    if (altitude < s_config_ptr->stage_min_ignite_altitude_m ||
        altitude > s_config_ptr->stage_max_ignite_altitude_m) {
        return FP_STG_NOGO;
    }

    if (angle_deg < s_config_ptr->stage_min_ignite_angle_deg ||
        angle_deg > s_config_ptr->stage_max_ignite_angle_deg) {
        return FP_STG_NOGO;
    }

    return FP_STG_GO;
}
