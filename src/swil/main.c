#include <stdint.h>
#include <stdio.h>

#include "flight_control.h"
#include "hwil/hwil.h"
#include "state_estimation.h"
#include "storage.h"
#include "timer.h"

const static char s_sensor_fname[] = "sim_out/sensor_native_sim.csv";
const static char s_state_fname[] = "sim_out/state_native_sim.csv";

int main() {
    // Need to do this because PAL_LOG* writes to FDs 3 and 4
    // NOTE: this is Windows-specific; change to /dev/null on Unix
    printf("Redirecting FD 3 and FD 4 to NUL\n");
    FILE* n1 = fopen("NUL", "w");
    FILE* n2 = fopen("NUL", "w");

    printf("Creating sensor output file at %s\n", s_sensor_fname);
    create_sensor_csv(s_sensor_fname);

    printf("Creating state output file at %s\n", s_state_fname);
    create_state_csv(s_state_fname);

    printf("***** Starting simulation *****\n");

    ASSERT_OK(se_init(), "failed to init state est\n");
    ASSERT_OK(fp_init(), "failed to init control logic\n");

    SensorFrame hwil_sensor_frame;
    while (get_hwil_sensor_frame(&hwil_sensor_frame) == STATUS_OK) {
        hwil_sensor_frame.timestamp = MICROS();

        store_sensor_frame(&hwil_sensor_frame);

        FlightPhase fp_before = fp_get();
        fp_update(&hwil_sensor_frame);
        if (fp_get() != fp_before) {
            printf("^ @ %.1f s\n\n", MILLIS() / 1000.);
        }

        StateFrame state_frame = se_as_frame();
        state_frame.flight_phase = fp_get();
        state_frame.timestamp = MICROS();
        store_state_frame(&state_frame);

        DELAY(10);
    }

    printf("***** Finished simulation *****\n");

    close_sensor_csv();

    return 0;
}
