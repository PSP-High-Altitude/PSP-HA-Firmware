#include "board_config.h"

#include <stdio.h>

#include "backup/backup.h"
#include "pyros.h"
#include "stdio.h"

static BoardConfig s_config = {
    .control_loop_period_ms = 10,             // ms
    .sensor_loop_period_ms = 100,             // ms
    .storage_loop_period_ms = 1000,           // ms
    .gps_loop_period_ms = 200,                // ms
    .pspcom_rx_loop_period_ms = 100,          // ms
    .pspcom_tx_ground_loop_period_ms = 5000,  // ms
    .pspcom_tx_flight_loop_period_ms = 1000,  // ms

    // State estimation settings
    .state_init_time_ms = 10000,    // ms
    .boost_detect_period_ms = 500,  // ms
    .launch_detect_replay = true,   // will replay
    .min_fast_vel_mps = 300,        // m/s
    .min_boost_acc_mps2 = 25,       // m/s^2
    .max_coast_acc_mps2 = 0,        // m/s^2
    .max_grounded_alt_m = 100,      // m
    .min_grounded_time_ms = 10000,  // ms

    // Stage separation settings
    .stage_is_separator_bool = 1,
    .stage_sep_lockout_ms = 4500,
    .stage_sep_delay_ms = 800,
    .stage_min_sep_velocity_mps = -1e9f,
    .stage_max_sep_velocity_mps = 1e9f,
    .stage_min_sep_altitude_m = 100,
    .stage_max_sep_altitude_m = 1e9f,
    .stage_min_sep_angle_deg = -1e4,
    .stage_max_sep_angle_deg = 1e4,
    .stage_sep_pyro_channel = PYRO_A1,

    // Stage ignititon settings
    .stage_is_igniter_bool = 0,
    .stage_ignite_lockout_ms = 5000,
    .stage_min_ignite_velocity_mps = 60.96f,
    .stage_max_ignite_velocity_mps = 1e6f,
    .stage_min_ignite_altitude_m = 100,
    .stage_max_ignite_altitude_m = 1e6f,
    .stage_min_ignite_angle_deg = 0,
    .stage_max_ignite_angle_deg = 0,
    .stage_ignite_pyro_channel = PYRO_A1,

    // Recovery settings
    .main_height_m = 300.0,      // m
    .drogue_delay_ms = 0,        // ms
    .deploy_lockout_ms = 10000,  // ms

    // Telemetry settings
    .telemetry_frequency_hz = 433000000,  // Hz
};

BoardConfig* config_get_ptr() { return &s_config; }

// Ensure that a valid config is loaded into SRAM
Status config_load() {
    printf("Config loaded\n");
    return STATUS_OK;
}

// Save changes to the config by updating checksum and storing to flash
Status config_commit() {
    printf("Config loaded\n");
    return STATUS_OK;
}

// Force next load to load from NAND by invalidating SRAM copy
Status config_invalidate() {
    printf("Config invalidated\n");
    return STATUS_OK;
}

void config_print() {
    BoardConfig* config = config_get_ptr();
    printf("===== BOARD CONFIG =====\n");

    printf("\n----- OS CONFIG -----\n");
    printf("Control loop period: %ld ms\n", config->control_loop_period_ms);
    printf("Sensor loop period: %ld ms\n", config->sensor_loop_period_ms);
    printf("Storage loop period: %ld ms\n", config->storage_loop_period_ms);
    printf("GPS loop period: %ld ms\n", config->gps_loop_period_ms);
    printf("PSPCOM RX loop period: %ld ms\n", config->pspcom_rx_loop_period_ms);
    printf("PSPCOM TX ground loop period: %ld ms\n",
           config->pspcom_tx_ground_loop_period_ms);
    printf("PSPCOM TX flight loop period: %ld ms\n",
           config->pspcom_tx_flight_loop_period_ms);

    printf("\n----- STATE ESTIMATION -----\n");
    printf("State init time: %ld ms\n", config->state_init_time_ms);
    printf("Launch detect period: %ld ms\n", config->boost_detect_period_ms);
    printf("Launch detect replay: %s\n",
           config->launch_detect_replay ? "Yes" : "No");
    printf("Min fast vel: %.2f m/s\n", config->min_fast_vel_mps);
    printf("Min boost acc: %.2f m/s^2\n", config->min_boost_acc_mps2);
    printf("Max coast acc: %.2f m/s^2\n", config->max_coast_acc_mps2);
    printf("Max grounded alt: %.2f m\n", config->max_grounded_alt_m);
    printf("Min grounded time: %.2f ms\n", config->min_grounded_time_ms);

    printf("\n----- STAGE SEPARATION -----\n");
    printf("Stage is separator: %s\n",
           config->stage_is_separator_bool ? "Yes" : "No");
    printf("Stage sep delay: %ld ms\n", config->stage_sep_lockout_ms);
    printf("Stage min sep vel: %.2f m/s\n", config->stage_min_sep_velocity_mps);
    printf("Stage max sep vel: %.2f m/s\n", config->stage_max_sep_velocity_mps);
    printf("Stage min sep alt: %.2f m\n", config->stage_min_sep_altitude_m);
    printf("Stage max sep alt: %.2f m\n", config->stage_max_sep_altitude_m);
    printf("Stage min sep angle: %.2f deg\n", config->stage_min_sep_angle_deg);
    printf("Stage max sep angle: %.2f deg\n", config->stage_max_sep_angle_deg);
    printf("Stage sep pyro channel: %ld\n", config->stage_sep_pyro_channel);

    printf("\n----- STAGE IGNITION -----\n");
    printf("Stage is igniter: %s\n",
           config->stage_is_igniter_bool ? "Yes" : "No");
    printf("Stage ignite delay: %ld ms\n", config->stage_ignite_lockout_ms);
    printf("Stage min ignite vel: %.2f m/s\n",
           config->stage_min_ignite_velocity_mps);
    printf("Stage max ignite vel: %.2f m/s\n",
           config->stage_max_ignite_velocity_mps);
    printf("Stage min ignite alt: %.2f m\n",
           config->stage_min_ignite_altitude_m);
    printf("Stage max ignite alt: %.2f m\n",
           config->stage_max_ignite_altitude_m);
    printf("Stage min ignite angle: %.2f deg\n",
           config->stage_min_ignite_angle_deg);
    printf("Stage max ignite angle: %.2f deg\n",
           config->stage_max_ignite_angle_deg);
    printf("Stage ignite pyro channel: %ld\n",
           config->stage_ignite_pyro_channel);

    printf("\n----- RECOVERY -----\n");
    printf("Main height: %.2f m\n", config->main_height_m);
    printf("Drogue delay: %ld ms\n", config->drogue_delay_ms);
    printf("Deploy lockout: %ld ms\n", config->deploy_lockout_ms);

    printf("\n----- TELEMETRY -----\n");
    printf("Telemetry frequency: %ld Hz\n", config->telemetry_frequency_hz);

    printf("\nChecksum: %08lx\n", config->checksum);
    printf("========================\n");
}
