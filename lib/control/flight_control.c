#include "flight_control.h"

#include <stdlib.h>

#include "backup.h"
#include "board_config.h"
#include "state_estimation.h"
#include "timer.h"

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
            return STATUS_MEMORY_ERROR;
        }
    }

    if (*s_flight_phase_ptr == FP_INIT || *s_flight_phase_ptr == FP_READY ||
        *s_flight_phase_ptr == FP_LANDED) {
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

        case FP_FAST_BOOST_1:
            *s_flight_phase_ptr = fp_update_fast_boost_1(sensor_frame);
            break;

        case FP_FAST_1:
            *s_flight_phase_ptr = fp_update_fast_1(sensor_frame);
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

        case FP_FAST_BOOST_2:
            *s_flight_phase_ptr = fp_update_fast_boost_2(sensor_frame);
            break;

        case FP_FAST_2:
            *s_flight_phase_ptr = fp_update_fast_2(sensor_frame);
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
            break;
    }

    return STATUS_OK;
}

FlightPhase fp_update_init(const SensorFrame* sensor_frame) {
    // During init, we're just updating the state and ignoring
    // the output, because at this point it won't be reliable
    uint64_t init_period = s_config_ptr->state_init_time_ms;
    if (MILLIS() - s_init_start_ms > init_period) {
        // Once the init time is up, move to ready
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
    static uint64_t s_last_frame_timestamp_ms;
    static uint64_t s_ms_accel_above_threshold = 0;

    static size_t s_ld_buffer_entries = 0;

    float accel_threshold = s_config_ptr->min_boost_acc_mps2;
    float accel_current = sensor_frame->acc_i_x * 9.81;

    if (accel_current > accel_threshold) {
        s_ms_accel_above_threshold += MILLIS() - s_last_frame_timestamp_ms;

        // If we need to replay launch, save the current frame in the buffer
        if (s_config_ptr->launch_detect_replay) {
            if (s_ld_buffer_entries < s_ld_buffer_size) {
                s_ld_buffer_data[s_ld_buffer_entries++] = *sensor_frame;
            }
        }
    } else {
        s_ms_accel_above_threshold = 0;
        s_ld_buffer_entries = 0;
    }

    uint64_t launch_detect_period = s_config_ptr->launch_detect_period_ms;
    if (s_ms_accel_above_threshold > launch_detect_period) {
        s_launch_time_ms = MILLIS();
        if (s_config_ptr->launch_detect_replay) {
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
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in boost 1\n");

    const StateEst* state = se_predict();

    uint8_t is_fast = state->velBody.x > s_config_ptr->min_fast_vel_mps;
    uint8_t is_coast = state->accBody.x < s_config_ptr->max_coast_acc_mps2;

    if (is_fast && is_coast) {
        PAL_LOGI("FP_BOOST_1 -> FP_FAST_1\n");
        return FP_FAST_1;
    }
    if (is_coast) {
        PAL_LOGI("FP_BOOST_1 -> FP_COAST_1\n");
        return FP_COAST_1;
    }
    if (is_fast) {
        PAL_LOGI("FP_BOOST_1 -> FP_FAST_BOOST_1\n");
        return FP_FAST_BOOST_1;
    }
    return FP_BOOST_1;
}

FlightPhase fp_update_fast_boost_1(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in fast boost 1\n");

    const StateEst* state = se_predict();

    uint8_t is_fast = state->velBody.x > s_config_ptr->min_fast_vel_mps;
    uint8_t is_coast = state->accBody.x < s_config_ptr->max_coast_acc_mps2;

    if (is_fast && is_coast) {
        return FP_FAST_1;
    }
    if (is_coast) {
        return FP_COAST_1;
    }
    if (is_fast) {
        return FP_FAST_BOOST_1;
    }
    return FP_BOOST_1;
}

FlightPhase fp_update_fast_1(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in fast 1\n");

    const StateEst* state = se_predict();

    FlightPhase sep_status = fp_stage_check_sep_lockout(sensor_frame, state);
    if (sep_status == FP_STG_GO) {
        return FP_STAGE;
    }

    FlightPhase ignite_status =
        fp_stage_check_ignite_lockout(sensor_frame, state);
    if (ignite_status == FP_STG_GO) {
        return FP_IGNITE;
    }

    uint8_t is_fast = state->velGeo.x > s_config_ptr->min_fast_vel_mps;
    if (sep_status == FP_STG_NOGO && ignite_status == FP_STG_NOGO) {
        return is_fast ? FP_FAST_2 : FP_COAST_2;
    }
    return is_fast ? FP_FAST_1 : FP_COAST_1;
}

FlightPhase fp_update_coast_1(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in coast 1\n");

    const StateEst* state = se_predict();

    FlightPhase sep_status = fp_stage_check_sep_lockout(sensor_frame, state);
    if (sep_status == FP_STG_GO) {
        return FP_STAGE;
    }

    FlightPhase ignite_status =
        fp_stage_check_ignite_lockout(sensor_frame, state);
    if (ignite_status == FP_STG_GO) {
        return FP_IGNITE;
    }

    if (sep_status == FP_STG_NOGO && ignite_status == FP_STG_NOGO) {
        return FP_COAST_2;
    }
    return FP_COAST_1;
}
FlightPhase fp_update_stage(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in staging\n");

    const StateEst* state = se_predict();

    // TODO: stage separation

    FlightPhase ignite_status =
        fp_stage_check_ignite_lockout(sensor_frame, state);

    if (ignite_status == FP_STG_GO) {
        return FP_IGNITE;
    }
    uint8_t is_fast = state->velGeo.x > s_config_ptr->min_fast_vel_mps;
    if (ignite_status == FP_STG_WAIT) {
        return is_fast ? FP_FAST_1 : FP_COAST_1;
    }
    return is_fast ? FP_FAST_2 : FP_COAST_2;
}
FlightPhase fp_update_ignite(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in staging\n");

    const StateEst* state = se_predict();

    // TODO: motor ignition

    uint8_t is_fast = state->velGeo.x > s_config_ptr->min_fast_vel_mps;
    return is_fast ? FP_FAST_BOOST_2 : FP_BOOST_2;
}
FlightPhase fp_update_boost_2(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in boost 2\n");

    const StateEst* state = se_predict();

    uint8_t is_fast = state->velBody.x > s_config_ptr->min_fast_vel_mps;
    uint8_t is_coast = state->accBody.x < s_config_ptr->max_coast_acc_mps2;

    if (is_fast && is_coast) {
        return FP_FAST_2;
    }
    if (is_coast) {
        return FP_COAST_2;
    }
    if (is_fast) {
        return FP_FAST_BOOST_2;
    }
    return FP_BOOST_2;
}
FlightPhase fp_update_fast_boost_2(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in fast boost 2\n");

    const StateEst* state = se_predict();

    uint8_t is_fast = state->velBody.x > s_config_ptr->min_fast_vel_mps;
    uint8_t is_coast = state->accBody.x < s_config_ptr->max_coast_acc_mps2;

    if (is_fast && is_coast) {
        return FP_FAST_2;
    }
    if (is_coast) {
        return FP_COAST_2;
    }
    if (is_fast) {
        return FP_FAST_BOOST_2;
    }
    return FP_BOOST_2;
}
FlightPhase fp_update_fast_2(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in fast 2\n");

    const StateEst* state = se_predict();

    uint8_t is_fast = state->velGeo.x > s_config_ptr->min_fast_vel_mps;
    return is_fast ? FP_FAST_2 : FP_COAST_2;
}
FlightPhase fp_update_coast_2(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in coast 2\n");

    const StateEst* state = se_predict();

    if (state->velBody.x < 0) {
        if (!s_apogee_time_ms) {
            s_apogee_time_ms = MILLIS();
        }
        if (s_apogee_time_ms + s_config_ptr->drogue_delay_ms < MILLIS()) {
            if (MILLIS() > s_config_ptr->deploy_lockout_ms) {
                // TODO: deploy drogue
                return FP_DROGUE;
            }
        }
    }

    uint8_t is_fast = state->velGeo.x > s_config_ptr->min_fast_vel_mps;
    return is_fast ? FP_FAST_2 : FP_COAST_2;
}
FlightPhase fp_update_drogue(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in drogue\n");

    const StateEst* state = se_predict();

    if (fp_check_grounded(sensor_frame, state)) {
        return FP_LANDED;
    }
    if (state->posBody.x < s_config_ptr->main_height_m) {
        // TODO: deploy main
        return FP_MAIN;
    }
    return FP_DROGUE;
}
FlightPhase fp_update_main(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in main\n");

    const StateEst* state = se_predict();

    if (fp_check_grounded(sensor_frame, state)) {
        return FP_LANDED;
    }
    return FP_MAIN;
}
FlightPhase fp_update_landed(const SensorFrame* sensor_frame) {
    // TODO: landed
    return FP_LANDED;
}

uint8_t fp_stage_check_sep_lockout(const SensorFrame* sensor_frame,
                                   const StateEst* state) {
    if (s_stage_sep_locked) {
        return FP_STG_NOGO;
    }
    if (!s_config_ptr->stage_is_separator_bool) {
        s_stage_sep_locked = 1;
        s_stage_sep_status = 0;
        return FP_STG_NOGO;
    }
    if (MILLIS() - s_launch_time_ms < s_config_ptr->stage_sep_delay_ms) {
        return FP_STG_WAIT;
    }

    // Since nothing else will cause a wait condition
    s_stage_sep_locked = 1;

    float velocity = state->velBody.x;
    float altitude = state->posBody.x;
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

uint8_t fp_stage_check_ignite_lockout(const SensorFrame* sensor_frame,
                                      const StateEst* state) {
    if (s_stage_ignite_locked) {
        return FP_STG_NOGO;
    }
    if (!s_config_ptr->stage_is_igniter_bool) {
        s_stage_ignite_locked = 1;
        s_stage_ignite_status = 0;
        return FP_STG_NOGO;
    }
    if (MILLIS() - s_launch_time_ms < s_config_ptr->stage_ignite_delay_ms) {
        return FP_STG_WAIT;
    }

    // Since nothing else will cause a wait condition
    s_stage_ignite_locked = 1;

    float velocity = state->velBody.x;
    float altitude = state->posBody.x;
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

uint8_t fp_check_grounded(const SensorFrame* sensorframe,
                          const StateEst* state) {
    return state->posBody.x < 10;
}
