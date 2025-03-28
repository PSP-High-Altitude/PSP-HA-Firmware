#include "control.h"

#include <math.h>

#include "board.h"
#include "board_config.h"
#include "flight_control.h"
#include "gpio/gpio.h"
#include "sensors.h"
#include "state.pb.h"
#include "state_estimation.h"
#include "storage.h"
#include "telem.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

/********************/
/* STATIC VARIABLES */
/********************/
static QueueHandle_t s_sensor_queue;

static BoardConfig* s_config_ptr;

static SensorFrame s_nan_frame = {
    .timestamp = 0,

    .acc_h_x = NAN,
    .acc_h_y = NAN,
    .acc_h_z = NAN,

    .acc_i_x = NAN,
    .acc_i_y = NAN,
    .acc_i_z = NAN,

    .mag_i_x = NAN,
    .mag_i_y = NAN,
    .mag_i_z = NAN,

    .rot_i_x = NAN,
    .rot_i_y = NAN,
    .rot_i_z = NAN,

    .temperature = NAN,
    .pressure = NAN,

};

/********************/
/* HELPER FUNCTIONS */
/********************/

/*****************/
/* API FUNCTIONS */
/*****************/
Status control_init() {
    ASSERT_OK(se_init(), "failed to init state est\n");
    ASSERT_OK(fp_init(), "failed to init control logic\n");

    // Each iteration of the control loop triggers one sensor read, so there
    // can't be more than one frame in the queue. The reason to use a queue is
    // for synchronization guarantees rather than actual buffering.
    s_sensor_queue = xQueueCreate(1, sizeof(SensorFrame));

    s_config_ptr = config_get_ptr();
    if (s_config_ptr == NULL) {
        ASSERT_OK(STATUS_STATE_ERROR, "unable to get ptr to config\n");
    }

    return STATUS_OK;
}

Status control_update_sensors(const SensorFrame* sensor_frame) {
    xQueueOverwrite(s_sensor_queue, sensor_frame);
    return STATUS_OK;
}

void task_control(TaskHandle_t* handle_ptr) {
    TickType_t last_iteration_start_tick = xTaskGetTickCount();

    while (1) {
        SensorFrame sensor_frame;
        Status update_status;

        // Check if we have sensor data available without waiting
        if (xQueueReceive(s_sensor_queue, &sensor_frame, 0) != pdPASS) {
            // If we didn't have new data available, send the NAN frame
            // but with an updated timestamp so that state estimation
            // can correctly compute the dt from adjacent iterations
            s_nan_frame.timestamp = MICROS();
            update_status = fp_update(&s_nan_frame);
        } else {
            // Otherwise just forward the frame as is
            update_status = fp_update(&sensor_frame);
        }

        // Trigger sensor read for next iteration
        sensors_start_read();

        // Store the state only if update returned OK
        if (update_status == STATUS_OK) {
            StateFrame state_frame = se_as_frame();
            state_frame.gentimestamp = MICROS();
            storage_queue_state(&state_frame);
        }

        // Get new state and send state updates where required
        FlightPhase flight_phase = fp_get();
        telem_update_fp(flight_phase);

        // Yellow LED is solid when READY, strobing when in flight
        if (flight_phase == FP_READY) {
            gpio_write(PIN_YELLOW, GPIO_HIGH);
        } else if (flight_phase > FP_READY && flight_phase < FP_LANDED) {
            // 25% duty cycle, 400 ms period
            gpio_write(PIN_YELLOW, (MILLIS() % 400) < 100);
        } else {
            gpio_write(PIN_YELLOW, GPIO_LOW);
        }

        // Blink red LED in error state
        if (flight_phase == FP_ERROR) {
            // 25% duty cycle, 400 ms period
            gpio_write(PIN_RED, (MILLIS() % 400) < 100);
        }

        // Periodically pause storage to segment files when grounded
        static uint64_t s_last_autosave_ms = 0;
        if (flight_phase == FP_WAIT || flight_phase == FP_LANDED) {
            // Start pausing every time we do this check if it has
            // been more than 5 minutes since the last time we did it
            if (MILLIS() - s_last_autosave_ms > 5 * 60 * 1000) {
                PAL_LOGI("Starting periodic grounded storage pause\n");
                s_last_autosave_ms = MILLIS();
                storage_pause(STORAGE_PAUSE_BRK);
            }
        }

        // If the storage is currently paused, clear our pause flag
        // This is okay because we're the only ones using this flag,
        // and the storage won't resume if someone else stopped it
        if (!storage_is_active()) {
            storage_start(STORAGE_PAUSE_BRK);
        }

        vTaskDelayUntil(&last_iteration_start_tick,
                        pdMS_TO_TICKS(s_config_ptr->control_loop_period_ms));
    }
}
