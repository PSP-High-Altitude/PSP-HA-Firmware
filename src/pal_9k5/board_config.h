#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "status.h"

typedef struct {
    // period in ms between state estimation update steps
    uint32_t sampling_period_ms;

    /* STATE ESTIMATION SETTINGS */
    // time in ms during which a baseline value for the sensors is determined
    uint32_t state_init_time_ms;
    // time in ms for which accel must be above boost threshold to detect launch
    uint32_t launch_detect_period_ms;
    // whether to replay state estimation of launch detection interval at exit
    bool launch_detect_replay;
    // vertical velocity over which we're considered to be fast
    float min_fast_vel_mps;
    // launch minimum acceleration above which we are considered to be in boost
    float min_boost_acc_mps2;
    // maximum acceleration below which we are considered to be in coast
    float max_coast_acc_mps2;

    /* RECOVERY SETTINGS */
    // height above ground in m at which main pyro is fired
    float main_height_m;
    // delay in ms from detecting apogee to firing drogue pyro
    uint32_t drogue_delay_ms;
    // time in ms from launch detection during which pyros cannot fire
    uint32_t deploy_lockout_ms;

    // CRC-32 checksum of the config
    uint32_t checksum;
} BoardConfig;

// Get a pointer to the config object
BoardConfig* get_config_ptr();

// Ensure that a valid config is loaded
Status load_config();

// Save the changes to the config
Status commit_config();

// Invalidate the config so it's reloaded from flash
Status invalidate_config();

#endif  // BOARD_CONFIG_H
