#include "state.h"

#include "data.h"
#include "storage.h"
#include "vector.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

/********************/
/* STATIC VARIABLES */
/********************/
// Do this with a queue for nice waiting semantics
static QueueHandle_t s_last_sensor_data_handle;

static Vector s_imu_up;
static Vector s_high_g_up;

static FlightPhase s_flight_phase;
static StateEst s_current_state;

static float s_acc_internal_buffer[MAX_AVG_BUFFER_SIZE];
static float s_baro_internal_buffer[MAX_AVG_BUFFER_SIZE];

static AverageBuffer s_acc_buffer = {.buffer = s_acc_internal_buffer,
                                     .size = MAX_AVG_BUFFER_SIZE};
static AverageBuffer s_baro_buffer = {.buffer = s_baro_internal_buffer,
                                      .size = MAX_AVG_BUFFER_SIZE};

static GPS_Fix_TypeDef s_last_gps_fix = {0};

/********************/
/* HELPER FUNCTIONS */
/********************/
static StateFrame state_data_to_pb_frame(uint64_t timestamp, FlightPhase fp,
                                         const StateEst* state) {
    StateFrame state_frame;

    state_frame.timestamp = timestamp;
    state_frame.flight_phase = fp;

    state_frame.pos_n = state->posNED.x;
    state_frame.pos_e = state->posNED.y;
    state_frame.pos_d = state->posNED.z;

    state_frame.vel_n = state->velNED.x;
    state_frame.vel_e = state->velNED.y;
    state_frame.vel_d = state->velNED.z;

    state_frame.acc_n = state->accNED.x;
    state_frame.acc_e = state->accNED.y;
    state_frame.acc_d = state->accNED.z;

    state_frame.vel_x = state->velBody.x;
    state_frame.vel_y = state->velBody.y;
    state_frame.vel_z = state->velBody.z;

    state_frame.acc_x = state->accBody.x;
    state_frame.acc_y = state->accBody.y;
    state_frame.acc_z = state->accBody.z;

    state_frame.orient_x = state->orientation.x;
    state_frame.orient_y = state->orientation.y;
    state_frame.orient_z = state->orientation.z;

    return state_frame;
}

/*****************/
/* API FUNCTIONS */
/*****************/
Status init_state_est() {
    // Single element queue since we always want to use the latest sensor data
    s_last_sensor_data_handle = xQueueCreate(1, sizeof(SensorFrame));

    return reset_state_est();
}

Status reset_state_est() {
    fp_init(&s_flight_phase, &s_current_state, &s_imu_up, &s_high_g_up,
            &s_acc_buffer, &s_baro_buffer);

    return STATUS_OK;
}

void update_latest_sensor_frame(SensorFrame* sensor_frame) {
    xQueueOverwrite(s_last_sensor_data_handle, sensor_frame);
}

void update_latest_gps_fix(GPS_Fix_TypeDef* gps_fix) {
    s_last_gps_fix = *gps_fix;
}

void state_est_task() {
    while (1) {
        // Receive the next state frame
        SensorFrame last_sensor_frame;
        if (xQueueReceive(s_last_sensor_data_handle, &last_sensor_frame,
                          pdMS_TO_TICKS(SENSOR_UPDATE_TIMEOUT_MS)) != pdPASS) {
            // If we don't receive a new sensor frame in a long time, something
            // has gone very wrong and the sensor task is probably stuck. We
            // can't really do much about it, so just invalidate our local copy
            // of the struct and continue hoping that we at least have GPS
            EXPECT_OK(STATUS_TIMEOUT_ERROR, "State est sensor receive");

            // (int)-1 is all 1s, which is (float)nan
            memset(&last_sensor_frame, -1, sizeof(last_sensor_frame));

            // We do want a realistic timestamp just in case
            last_sensor_frame.timestamp = MICROS();
        }

        // GPS will have updated asynchronously, so no need to wait
        fp_update(&last_sensor_frame, &s_last_gps_fix, &s_flight_phase,
                  &s_current_state, &s_imu_up, &s_high_g_up, &s_acc_buffer,
                  &s_baro_buffer);

        // Send the updated state to the store queue
        StateFrame state_frame =
            state_data_to_pb_frame(MICROS(), s_flight_phase, &s_current_state);
        queue_state_store(&state_frame);
    }
}

FlightPhase* get_last_flight_phase() { return &s_flight_phase; }