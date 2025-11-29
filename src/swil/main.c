#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "board_config.h"
#include "flight_control.h"
#include "hwil/hwil.h"
#include "state_estimation.h"
#include "storage.h"
#include "timer.h"

const static char s_sensor_fname[] = "sim_out/sensor.csv";
const static char s_state_fname[] = "sim_out/state.csv";

int main() {
    // Need to do this because PAL_LOG* writes to FDs 3 and 4
    // NOTE: this is Windows-specific; change to /dev/null on Unix
    printf("Redirecting FD 3 and FD 4 to NUL\n");
    __attribute__((unused)) FILE* n1 = fopen("NUL", "w");
    __attribute__((unused)) FILE* n2 = fopen("NUL", "w");

    printf("Creating sensor output file at %s\n", s_sensor_fname);
    create_sensor_csv(s_sensor_fname);

    printf("Creating state output file at %s\n", s_state_fname);
    create_state_csv(s_state_fname);

    float max_acc = 0;
    float max_vel = 0;
    float max_alt = 0;

    printf("\n***** Starting simulation *****\n\n");

    ASSERT_OK(se_init(), "failed to init state est\n");
    ASSERT_OK(fp_init(), "failed to init control logic\n");
    printf("^ @ %.1f s\n\n", MILLIS() / 1000.);

    SensorFrame hwil_sensor_frame;
    while (get_hwil_sensor_frame(&hwil_sensor_frame) == STATUS_OK) {
        hwil_sensor_frame.timestamp = MICROS();
        store_sensor_frame(&hwil_sensor_frame);

        FlightPhase fp_before = fp_get();
        Status update_status = fp_update(&hwil_sensor_frame);
        if (fp_get() != fp_before) {
            printf("^ @ %.1f s\n\n", MILLIS() / 1000.);
        }

        if (update_status == STATUS_OK) {
            StateFrame state_frame = se_as_frame();
            state_frame.gentimestamp = MICROS();
            store_state_frame(&state_frame);

            max_acc = fmaxf(max_acc, state_frame.acc_vert);
            max_vel = fmaxf(max_vel, state_frame.vel_vert);

            if (fp_get() == FP_DROGUE && fp_before == FP_COAST) {
                max_alt = state_frame.pos_ekf;
            }
        }

        DELAY(config_get_ptr()->control_loop_period_ms);
    }

    printf("End of input data\n");
    printf("^ @ %.1f s\n\n", MILLIS() / 1000.);

    printf("***** Finished simulation *****\n\n");

    printf("Max acc: %.2f m/s^2\n", max_acc);
    printf("Max vel: %.2f m/s\n", max_vel);
    printf("Max alt: %.2f m\n", max_alt);

    close_sensor_csv();
    close_state_csv();

    return 0;
}
