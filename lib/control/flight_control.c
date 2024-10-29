#include "flight_control.h"

#include <math.h>
#include <stdlib.h>

#include "backup/backup.h"
#include "board_config.h"
#include "pyros.h"
#include "state_estimation.h"
#include "timer.h"

// Forward declarations
FlightPhase fp_update_init(const SensorFrame* sensor_frame);
FlightPhase fp_update_ready(const SensorFrame* sensor_frame);
FlightPhase fp_update_boost_1(const SensorFrame* sensor_frame);
FlightPhase fp_update_coast_1(const SensorFrame* sensor_frame);
FlightPhase fp_update_stage(const SensorFrame* sensor_frame);
FlightPhase fp_update_ignite(const SensorFrame* sensor_frame);
FlightPhase fp_update_boost_2(const SensorFrame* sensor_frame);
FlightPhase fp_update_coast_2(const SensorFrame* sensor_frame);
FlightPhase fp_update_drogue(const SensorFrame* sensor_frame);
FlightPhase fp_update_main(const SensorFrame* sensor_frame);
FlightPhase fp_update_landed(const SensorFrame* sensor_frame);

FlightStageStatus fp_stage_check_sep_lockout(const SensorFrame* sensor_frame,
                                             const StateEst* state);
FlightStageStatus fp_stage_check_ignite_lockout(const SensorFrame* sensor_frame,
                                                const StateEst* state);

// Pointer to flight phase
static FlightPhase* s_flight_phase_ptr = NULL;

// Pointer to config object
static BoardConfig* s_config_ptr = NULL;

// Initialization start time
static uint64_t s_init_start_ms;

// Launch detect sensor data buffer
static SensorFrame* s_ld_buffer_data;
static size_t s_ld_buffer_size = 0;

static uint64_t s_launch_time_ms;
static uint64_t s_apogee_time_ms;
static uint8_t s_stage_sep_locked;
static uint8_t s_stage_ignite_locked;
static uint8_t s_stage_sep_status;
static uint8_t s_stage_ignite_status;

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
        s_ld_buffer_size = 1 + (s_config_ptr->launch_detect_period_ms /
                                s_config_ptr->control_loop_period_ms);

        s_ld_buffer_data = malloc(sizeof(SensorFrame) * s_ld_buffer_size);

        if (s_ld_buffer_data == NULL) {
            ASSERT_OK(STATUS_MEMORY_ERROR,
                      "failed to allocate launch replay buffer");
        }

        PAL_LOGI("Allocated %u entry buffer for launch replay\n",
                 s_ld_buffer_size);
    }

#ifdef HWIL_TEST
    // Always start from init for HWIL test
    *s_flight_phase_ptr = FP_INIT;
#endif

    if (*s_flight_phase_ptr == FP_INIT || *s_flight_phase_ptr == FP_READY ||
        *s_flight_phase_ptr == FP_LANDED || *s_flight_phase_ptr > FP_LANDED) {
        PAL_LOGI("Resetting flight logic and state estimation\n");
        ASSERT_OK(se_reset(), "failed to reset state\n");
        *s_flight_phase_ptr = FP_INIT;
    }

    s_apogee_time_ms = 0;
    s_stage_sep_locked = 0;
    s_stage_ignite_locked = 0;
    s_stage_sep_status = FP_STG_WAIT;
    s_stage_ignite_status = FP_STG_WAIT;

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

        case FP_READY:
            *s_flight_phase_ptr = fp_update_ready(sensor_frame);
            break;

        case FP_BOOST_1:
            *s_flight_phase_ptr = fp_update_boost_1(sensor_frame);
            break;

        case FP_COAST_1:
            *s_flight_phase_ptr = fp_update_coast_1(sensor_frame);
            break;

        case FP_STAGE:
            *s_flight_phase_ptr = fp_update_stage(sensor_frame);
            break;

        case FP_IGNITE:
            *s_flight_phase_ptr = fp_update_ignite(sensor_frame);
            break;

        case FP_BOOST_2:
            *s_flight_phase_ptr = fp_update_boost_2(sensor_frame);
            break;

        case FP_COAST_2:
            *s_flight_phase_ptr = fp_update_coast_2(sensor_frame);
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
    // determine the ground altitude later on
    static float s_pressure_sum = 0;
    static size_t s_pressure_num_pts = 0;

    if (!isnan(sensor_frame->pressure)) {
        s_pressure_sum += sensor_frame->pressure;
        s_pressure_num_pts += 1;
    }

    uint64_t init_period = s_config_ptr->state_init_time_ms;
    if (MILLIS() - s_init_start_ms > init_period) {
        // Once the init time is up, record the average ground pressure
        se_set_ground_pressure(s_pressure_sum / s_pressure_num_pts);

        PAL_LOGI("FP_INIT -> FP_READY\n");
        return FP_READY;
    }

    // Otherwise, stay in init
    return FP_INIT;
}

FlightPhase fp_update_ready(const SensorFrame* sensor_frame) {
    // In ready, we're tracking the acceleration for launch
    // detection and buffering data so that we can replay
    // it through the state estimation when launch is detected
    static uint64_t s_last_frame_timestamp_ms = 0;
    static uint64_t s_ms_accel_above_threshold = 0;

    static size_t s_ld_buffer_entries = 0;

    float accel_threshold = s_config_ptr->min_boost_acc_mps2;
    float accel_current = sensor_frame->acc_i_z * 9.81 - G_MAG;

    if (accel_current > accel_threshold) {
        s_ms_accel_above_threshold += MILLIS() - s_last_frame_timestamp_ms;

        // If we need to replay launch, save the current frame in the buffer
        if (s_config_ptr->launch_detect_replay) {
            if (s_ld_buffer_entries < s_ld_buffer_size) {
                s_ld_buffer_data[s_ld_buffer_entries++] = *sensor_frame;
            }
        }
    } else if (!isnan(accel_current)) {
        // Don't reset for a NAN acceleration
        s_ms_accel_above_threshold = 0;
        s_ld_buffer_entries = 0;
    }

    // Record this iteration time for future use
    s_last_frame_timestamp_ms = MILLIS();

    uint64_t launch_detect_period = s_config_ptr->launch_detect_period_ms;
    if (s_ms_accel_above_threshold > launch_detect_period) {
        s_launch_time_ms = MILLIS() - launch_detect_period;

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
        PAL_LOGI("FP_READY -> FP_BOOST_1\n");
        return FP_BOOST_1;
    }

    return FP_READY;
}

FlightPhase fp_update_boost_1(const SensorFrame* sensor_frame) {
    const StateEst* state = se_predict();

    uint8_t is_coast = state->accVert < s_config_ptr->max_coast_acc_mps2;

    if (is_coast) {
        PAL_LOGI("FP_BOOST_1 -> FP_COAST_1\n");
        return FP_COAST_1;
    }

    return FP_BOOST_1;
}

FlightPhase fp_update_coast_1(const SensorFrame* sensor_frame) {
    const StateEst* state = se_predict();

    FlightStageStatus sep_status =
        fp_stage_check_sep_lockout(sensor_frame, state);
    // Separate first if GO
    if (sep_status == FP_STG_GO) {
        EXPECT_OK(pyros_fire(s_config_ptr->stage_sep_pyro_channel),
                  "failed to fire stage sep pyro\n");

        PAL_LOGI("FP_COAST_1 -> FP_STAGE\n");
        return FP_STAGE;
    }

    FlightStageStatus ignite_status =
        fp_stage_check_ignite_lockout(sensor_frame, state);
    // Ignite otherwise if GO
    if (ignite_status == FP_STG_GO) {
        EXPECT_OK(pyros_fire(s_config_ptr->stage_ignite_pyro_channel),
                  "failed to fire motor ignitor\n");

        PAL_LOGI("FP_COAST_1 -> FP_IGNITE\n");
        return FP_IGNITE;
    }

    // If NOGO on both, go to coast 2 and don't try again
    if (sep_status == FP_STG_NOGO && ignite_status == FP_STG_NOGO) {
        PAL_LOGI("FP_COAST_1 -> FP_COAST_2\n");
        return FP_COAST_2;
    }

    // If WAIT on either, stay in coast 1 so we can try again
    return FP_COAST_1;
}

FlightPhase fp_update_stage(const SensorFrame* sensor_frame) {
    const StateEst* state = se_predict();

    FlightStageStatus ignite_status =
        fp_stage_check_ignite_lockout(sensor_frame, state);

    // Ignite if GO
    if (ignite_status == FP_STG_GO) {
        EXPECT_OK(pyros_fire(s_config_ptr->stage_ignite_pyro_channel),
                  "failed to fire motor ignitor\n");

        PAL_LOGI("FP_STAGE -> FP_IGNITE\n");
        return FP_IGNITE;
    }

    // If WAIT, stay in stage and try again
    if (ignite_status == FP_STG_WAIT) {
        return FP_STAGE;
    }

    // If NOGO, go to coast 2 and don't try again
    PAL_LOGI("FP_STAGE -> FP_COAST_2\n");
    return FP_COAST_2;
}

FlightPhase fp_update_ignite(const SensorFrame* sensor_frame) {
    PAL_LOGI("FP_IGNITE -> FP_BOOST_2\n");
    return FP_BOOST_2;
}

FlightPhase fp_update_boost_2(const SensorFrame* sensor_frame) {
    const StateEst* state = se_predict();

    uint8_t is_coast = state->accVert < s_config_ptr->max_coast_acc_mps2;

    if (is_coast) {
        PAL_LOGI("FP_BOOST_2 -> FP_COAST_2\n");
        return FP_COAST_2;
    }

    return FP_BOOST_2;
}

FlightPhase fp_update_coast_2(const SensorFrame* sensor_frame) {
    const StateEst* state = se_predict();

    if (state->velVert < 0) {
        if (!s_apogee_time_ms) {
            s_apogee_time_ms = MILLIS();
        }

        if (s_apogee_time_ms + s_config_ptr->drogue_delay_ms < MILLIS()) {
            if (MILLIS() > s_config_ptr->deploy_lockout_ms) {
                EXPECT_OK(pyros_fire(PYRO_DRG), "failed to fire drogue pyro\n");

                PAL_LOGI("FP_COAST_2 -> FP_DROGUE\n");
                return FP_DROGUE;
            }
        }
    }

    return FP_COAST_2;
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

    static uint64_t s_last_frame_timestamp_ms = 0;
    static uint64_t s_ms_alt_below_threshold = 0;

    if (state->posVert < s_config_ptr->max_grounded_alt_m) {
        s_ms_alt_below_threshold += MILLIS() - s_last_frame_timestamp_ms;
    } else {
        s_ms_alt_below_threshold = 0;
    }

    s_last_frame_timestamp_ms = MILLIS();

    if (s_ms_alt_below_threshold > s_config_ptr->min_grounded_time_ms) {
        PAL_LOGI("FP_MAIN -> FP_LANDED\n");
        return FP_LANDED;
    }

    return FP_MAIN;
}

FlightPhase fp_update_landed(const SensorFrame* sensor_frame) {
    // TODO: landed
    return FP_LANDED;
}

FlightStageStatus fp_stage_check_sep_lockout(const SensorFrame* sensor_frame,
                                             const StateEst* state) {
    if (s_stage_sep_locked) {
        return FP_STG_NOGO;
    }

    if (!s_config_ptr->stage_is_separator_bool) {
        s_stage_sep_locked = 1;
        s_stage_sep_status = 0;
        return FP_STG_NOGO;
    }

    if (MILLIS() - s_launch_time_ms < s_config_ptr->stage_sep_lockout_ms) {
        return FP_STG_WAIT;
    }

    // Since nothing else will cause a wait condition
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

    s_stage_sep_status = 1;
    return FP_STG_GO;
}

FlightStageStatus fp_stage_check_ignite_lockout(const SensorFrame* sensor_frame,
                                                const StateEst* state) {
    if (s_stage_ignite_locked) {
        return FP_STG_NOGO;
    }

    if (!s_config_ptr->stage_is_igniter_bool) {
        s_stage_ignite_locked = 1;
        s_stage_ignite_status = 0;
        return FP_STG_NOGO;
    }

    if (MILLIS() - s_launch_time_ms < s_config_ptr->stage_ignite_lockout_ms) {
        return FP_STG_WAIT;
    }

    // Since nothing else will cause a wait condition
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

    s_stage_ignite_status = 1;
    return FP_STG_GO;
}
