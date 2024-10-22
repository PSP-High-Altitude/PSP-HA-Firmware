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
        if (s_config_ptr->launch_detect_replay) {
            for (int i = 0; i < s_ld_buffer_entries; i++) {
                EXPECT_OK(se_update(*s_flight_phase_ptr, &s_ld_buffer_data[i]),
                          "state est update failed during launch replay\n");
            }
        }
        return FP_BOOST_1;
    }

    return FP_READY;
}

FlightPhase fp_update_boost_1(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in boost\n");

    const StateEst* state = se_predict();

    uint8_t is_fast = state->velGeo.x > s_config_ptr->min_fast_vel_mps;
    uint8_t is_coast = state->accGeo.x < s_config_ptr->max_coast_acc_mps2;

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

FlightPhase fp_update_fast_boost_1(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in fast boost\n");

    const StateEst* state = se_predict();

    uint8_t is_fast = state->velGeo.x > s_config_ptr->min_fast_vel_mps;
    uint8_t is_coast = state->accGeo.x < s_config_ptr->max_coast_acc_mps2;

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
    // TODO: fast_1
    return FP_FAST_1;
}

FlightPhase fp_update_coast_1(const SensorFrame* sensor_frame) {
    // TODO: coast_1
    return FP_COAST_1;
}
FlightPhase fp_update_stage(const SensorFrame* sensor_frame) {
    // TODO: stage
    return FP_STAGE;
}
FlightPhase fp_update_ignite(const SensorFrame* sensor_frame) {
    // TODO: ignite
    return FP_IGNITE;
}
FlightPhase fp_update_boost_2(const SensorFrame* sensor_frame) {
    // TODO: boost_2
    return FP_BOOST_2;
}
FlightPhase fp_update_fast_boost_2(const SensorFrame* sensor_frame) {
    // TODO: fast_boost_2
    return FP_FAST_BOOST_2;
}
FlightPhase fp_update_fast_2(const SensorFrame* sensor_frame) {
    // TODO: fast_2
    return FP_FAST_2;
}
FlightPhase fp_update_coast_2(const SensorFrame* sensor_frame) {
    // TODO: coast_2
    return FP_COAST_2;
}
FlightPhase fp_update_drogue(const SensorFrame* sensor_frame) {
    // TODO: drogue
    return FP_DROGUE;
}
FlightPhase fp_update_main(const SensorFrame* sensor_frame) {
    // TODO: main
    return FP_MAIN;
}
FlightPhase fp_update_landed(const SensorFrame* sensor_frame) {
    // TODO: landed
    return FP_LANDED;
}
