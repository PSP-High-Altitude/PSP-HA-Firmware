#include <stdio.h>

#include "board.h"
#include "board_config.h"
#include "gps.pb.h"
#include "i2c/i2c.h"
#include "max_m10s.h"
#include "pspcom.h"
#include "storage.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

#ifdef HWIL_TEST
#include "hwil/hwil.h"
#endif

/*********************/
/* PERIPHERAL CONFIG */
/*********************/

// MS5637 Barometer
static I2cDevice s_gps_conf = {
    .address = 0x42,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C2,
    .scl = PIN_PB10,
    .sda = PIN_PB11,
};

/********************/
/* STATIC VARIABLES */
/********************/
static TaskHandle_t* s_handle_ptr = NULL;
static BoardConfig* s_config_ptr = NULL;

/********************/
/* HELPER FUNCTIONS */
/********************/
GpsFrame gps_fix_to_pb_frame(uint64_t timestamp,
                             const GPS_Fix_TypeDef* gps_fix) {
    GpsFrame gps_frame;

    // Copy UTC Time
    gps_frame.timestamp = timestamp;
    gps_frame.year = gps_fix->year;
    gps_frame.month = gps_fix->month;
    gps_frame.day = gps_fix->day;
    gps_frame.hour = gps_fix->hour;
    gps_frame.min = gps_fix->min;
    gps_frame.sec = gps_fix->sec;

    // Pack validity flags into a single uint64_t
    gps_frame.valid_flags = ((uint64_t)gps_fix->date_valid << 0) |
                            ((uint64_t)gps_fix->time_valid << 1) |
                            ((uint64_t)gps_fix->time_resolved << 2) |
                            ((uint64_t)gps_fix->fix_type << 3) |
                            ((uint64_t)gps_fix->fix_valid << 8) |
                            ((uint64_t)gps_fix->diff_used << 9) |
                            ((uint64_t)gps_fix->psm_state << 10) |
                            ((uint64_t)gps_fix->hdg_veh_valid << 14) |
                            ((uint64_t)gps_fix->carrier_phase << 15) |
                            ((uint64_t)gps_fix->invalid_llh << 19);

    // Copy Navigation info
    gps_frame.num_sats = gps_fix->num_sats;
    gps_frame.lon = gps_fix->lon;
    gps_frame.lat = gps_fix->lat;
    gps_frame.height = gps_fix->height;
    gps_frame.height_msl = gps_fix->height_msl;
    gps_frame.accuracy_horiz = gps_fix->accuracy_horiz;
    gps_frame.accuracy_vertical = gps_fix->accuracy_vertical;
    gps_frame.vel_north = gps_fix->vel_north;
    gps_frame.vel_east = gps_fix->vel_east;
    gps_frame.vel_down = gps_fix->vel_down;
    gps_frame.ground_speed = gps_fix->ground_speed;
    gps_frame.hdg = gps_fix->hdg;
    gps_frame.accuracy_speed = gps_fix->accuracy_speed;
    gps_frame.accuracy_hdg = gps_fix->accuracy_hdg;

    return gps_frame;
}

/*****************/
/* API FUNCTIONS */
/*****************/
Status gps_init() {
    ASSERT_OK(max_m10s_init(&s_gps_conf), "failed to init GPS\n");

    s_config_ptr = config_get_ptr();
    if (s_config_ptr == NULL) {
        ASSERT_OK(STATUS_STATE_ERROR, "unable to get ptr to config\n");
    }

    return STATUS_OK;
}

void task_gps(TaskHandle_t* handle_ptr) {
    s_handle_ptr = handle_ptr;
    TickType_t last_iteration_start_tick = xTaskGetTickCount();

    while (1) {
        GPS_Fix_TypeDef fix;
        if (max_m10s_poll_fix(&s_gps_conf, &fix) == STATUS_OK) {
            GpsFrame gps_frame = gps_fix_to_pb_frame(MILLIS(), &fix);

            // Set LED to indicate GPS fix
            gpio_write(PIN_BLUE, (fix.fix_valid && !fix.invalid_llh)
                                     ? GPIO_HIGH
                                     : GPIO_LOW);

            // update_gps_for_control(&gps_frame);
            storage_queue_gps(&gps_frame);
            pspcom_update_gps(&fix);
        } else {
            // Set LED low
            gpio_write(PIN_BLUE, GPIO_LOW);
        }

        // Delay until next time
        vTaskDelayUntil(&last_iteration_start_tick,
                        pdMS_TO_TICKS(s_config_ptr->gps_loop_period_ms));
    }
}
