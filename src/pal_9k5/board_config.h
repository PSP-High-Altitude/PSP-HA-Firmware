#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "status.h"

#define CONFIG_DIR "/config"
#define CONFIG_FILENAME "board.conf"

typedef struct {
    // period in ms between state estimation update steps
    uint32_t control_loop_period_ms;
    // period in ms between sensor reads in the absence of control requests
    uint32_t sensor_loop_period_ms;
    // period in ms between file system flushes and pause request checks
    uint32_t storage_loop_period_ms;
    // period in ms between polling the GPS
    uint32_t gps_loop_period_ms;
    // period in ms between checking for incoming telemetry messages
    uint32_t pspcom_rx_loop_period_ms;
    // period in ms between sending standard telemetry message on the ground
    uint32_t pspcom_tx_ground_loop_period_ms;
    // period in ms between sending standard telemetry message in flight
    uint32_t pspcom_tx_flight_loop_period_ms;

    /* STATE ESTIMATION SETTINGS */
    // time in ms during which a baseline value for the sensors is determined
    uint32_t state_init_time_ms;
    // time in ms for which accel must be above boost threshold to detect launch
    uint32_t launch_detect_period_ms;
    // whether to replay state estimation of launch detection interval at exit
    uint32_t launch_detect_replay;
    // vertical velocity over which we're considered to be fast
    float min_fast_vel_mps;
    // launch minimum acceleration above which we are considered to be in boost
    float min_boost_acc_mps2;
    // maximum acceleration below which we are considered to be in coast
    float max_coast_acc_mps2;
    // maximum altitude under which we are considered to be grounded
    float max_grounded_alt_m;
    // minimum time for which we have to be below the grounded alt to transition
    float min_grounded_time_ms;

    /* STAGE SEPARATION SETTINGS */
    // status of this board as a stage separator
    // (1 to fire separation charge, 0 to not)
    uint8_t stage_is_separator_bool;
    // lockout after launch in ms to separate stages (NOT TO IGNITE)
    uint32_t stage_sep_lockout_ms;
    // minimum velocity in m/s to separate stages
    float stage_min_sep_velocity_mps;
    // maximum velocity in m/s to separate stages
    float stage_max_sep_velocity_mps;
    // minimum altitude in m to separate stages
    float stage_min_sep_altitude_m;
    // maximum altitude in m to separate stages
    float stage_max_sep_altitude_m;
    // minimum angle from vertical in deg to separate stages
    float stage_min_sep_angle_deg;
    // maximum angle from vertical in deg to separate stages
    float stage_max_sep_angle_deg;
    // stage separation pyro channel
    uint32_t stage_sep_pyro_channel;

    /* STAGE IGNITION SETTINGS */
    // status of this board as a MOTOR IGNITER
    // (1 to IGNITE motor after separation, 0 to NOT IGNITE)
    uint8_t stage_is_igniter_bool;
    // lockout after launch in ms to IGNITE the next stage
    uint32_t stage_ignite_lockout_ms;
    // minimum velocity in m/s to IGNITE stage
    float stage_min_ignite_velocity_mps;
    // maximum velocity in m/s to IGNITE stage
    float stage_max_ignite_velocity_mps;
    // minimum altitude in m to IGNITE stage
    float stage_min_ignite_altitude_m;
    // maximum altitude in m to IGNITE stage
    float stage_max_ignite_altitude_m;
    // minimum angle from vertical in deg to IGNITE stage
    float stage_min_ignite_angle_deg;
    // maximum angle from vertical in deg to IGNITE stage
    float stage_max_ignite_angle_deg;
    // stage separation pyro channel
    uint32_t stage_ignite_pyro_channel;

    /* RECOVERY SETTINGS */
    // height above ground in m at which main pyro is fired
    float main_height_m;
    // delay in ms from detecting apogee to firing drogue pyro
    uint32_t drogue_delay_ms;
    // time in ms from launch detection during which pyros cannot fire
    uint32_t deploy_lockout_ms;

    /* TELEMETRY SETTINGS */
    // Radio frequency in Hz at which telemetry is sent and received
    uint32_t telemetry_frequency_hz;

    // CRC-32 checksum of the config
    uint32_t checksum;
} BoardConfig;

// Get a pointer to the config object
BoardConfig* config_get_ptr();

// Ensure that a valid config is loaded
Status config_load();

// Save the changes to the config
Status config_commit();

// Invalidate the config so it's reloaded from flash
Status config_invalidate();

// Print the config
void config_print();

#endif  // BOARD_CONFIG_H
