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

        case FP_BOOST:
            *s_flight_phase_ptr = fp_update_boost(sensor_frame);
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
        return FP_BOOST;
    }

    return FP_READY;
}

FlightPhase fp_update_boost(const SensorFrame* sensor_frame) {
    EXPECT_OK(se_update(*s_flight_phase_ptr, sensor_frame),
              "state est update failed in boost\n");

    const StateEst* state = se_predict();

    if (-state->velGeo.z > s_config_ptr->min_fast_vel_mps) {
        return FP_FAST;
    }

    if (-state->accGeo.z < s_config_ptr->max_coast_acc_mps2) {
        return FP_COAST;
    }

    return FP_BOOST;
}
