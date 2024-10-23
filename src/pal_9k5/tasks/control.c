#include "control.h"

#include "board_config.h"
#include "flight_control.h"
#include "pspcom.h"
#include "sensors.h"
#include "state.pb.h"
#include "state_estimation.h"
#include "storage.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

/********************/
/* STATIC VARIABLES */
/********************/
static QueueHandle_t s_sensor_queue;

static BoardConfig* s_config_ptr;

/********************/
/* HELPER FUNCTIONS */
/********************/

/*****************/
/* API FUNCTIONS */
/*****************/
Status control_init() {
    ASSERT_OK(fp_init(), "failed to init control logic\n");
    // ASSERT_OK(se_init(), "failed to init state est\n");

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

        // Check if we have sensor data available without waiting
        if (xQueueReceive(s_sensor_queue, &sensor_frame, 0) != pdPASS) {
            // We didn't have new data available
            fp_update(NULL);
        } else {
            fp_update(&sensor_frame);
        }

        sensors_start_read();
        pspcom_update_fp(fp_get());

        StateFrame state_frame = se_as_frame();
        state_frame.flight_phase = fp_get();
        state_frame.timestamp = MICROS();
        storage_queue_state(&state_frame);

        vTaskDelayUntil(&last_iteration_start_tick,
                        pdMS_TO_TICKS(s_config_ptr->control_loop_period_ms));
    }
}
