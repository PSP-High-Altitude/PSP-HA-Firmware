#ifndef COMMANDS_H
#define COMMANDS_H

#include "backup/backup.h"
#include "board_config.h"
#include "mt29f4g.h"
#include "pspcom.h"
#include "regex.h"
#include "rtc/rtc.h"
#include "status.h"
#include "timer.h"

// Help command
char regex_help[] = "^help[\n]*$";
// clang-format off
void cmd_help(char *str) {
    printf(
        "Commands:\n"
        "  help                                  this command\n"
        "  set_datetime YYYY-MM-DD HH:MM:SS      sets the RTC time\n"
        "  get_datetime                          gets the RTC time\n"
        "  print_config                          prints the config\n"
        "  invalidate_config                     invalidates the config\n"
        "  invalidate_backup                     invalidates the backup pointer\n"
        "  erase_flash_chip                      full block-level flash erase\n"
        "  set_frequency [frequency in Hz]       sets the frequency\n"
        "  set_config_value [key] [value]        sets a config value\n"
        "  get_firmware_spec                     prints the firmware spec\n");
}
// clang-format on

// Set datetime command
char regex_set_datetime[] =
    "^set_datetime [0-9]{4}\\-[0-9]{2}\\-[0-9]{2} "
    "[0-9]{2}:[0-9]{2}:[0-9]{2}[\n]*$";
void cmd_set_datetime(char *str) {
    int year, month, day, hour, minute, second;
    sscanf(str, "set_datetime %d-%d-%d %d:%d:%d", &year, &month, &day, &hour,
           &minute, &second);
    RTCDateTime dt = {year, month, day, hour, minute, second};
    rtc_set_datetime(dt);
    PAL_LOGI("Time set!\n");
}

// Get datetime command
char regex_get_datetime[] = "^get_datetime[\n]*$";
void cmd_get_datetime(char *str) {
    RTCDateTime dt = rtc_get_datetime();
    printf("Current time: %04ld-%02ld-%02ld %02ld:%02ld:%02ld\n", dt.year,
           dt.month, dt.day, dt.hour, dt.minute, dt.second);
}

// Print config command
char regex_print_config[] = "^print_config[\n]*$";
void cmd_print_config(char *str) { config_print(); }

// Config invalidate command
char regex_invalidate_config[] = "^invalidate_config[\n]*$";
void cmd_invalidate_config(char *str) {
    config_invalidate();
    PAL_LOGW("Board config invalidated!\n");
}

// Backup invalidate command
char regex_invalidate_backup[] = "^invalidate_backup[\n]*$";
void cmd_invalidate_backup(char *str) {
    memset(backup_get_ptr(), 0, sizeof(Backup));
    PAL_LOGW("Board backup pointer invalidated!\n");
    DELAY_MICROS(1000);
    NVIC_SystemReset();
}

// Config erase chip
char regex_erase_flash_chip[] = "^erase_flash_chip[\n]*$";
void cmd_erase_flash_chip(char *str) {
    if (mt29f4g_erase_chip() == STATUS_OK) {
        PAL_LOGI("Successfully erased flash chip\n");
    } else {
        PAL_LOGE("Failed to erase flash chip\n");
    }
}

// Config set frequency command
char regex_set_frequency[] = "^set_frequency [0-9]{1,10}[\n]*$";
void cmd_set_frequency(char *str) {
    uint32_t frequency;
    sscanf(str, "set_frequency %ld", &frequency);
    if (pspcom_change_frequency(frequency) != STATUS_OK) {
        PAL_LOGE("Failed to change telemetry frequency!\n");
        return;
    }
    PAL_LOGW("Changed telemetry frequency to %.3f MHz!\n", frequency / 1e6);
}

// Config set frequency command
const char *boardConfigDescription;
char regex_set_config_value[] = "^set_config_value.*$";
void cmd_set_config_value(char *str) {
    char key[256];
    char value[256];
    int ret = sscanf(str, "set_config_value %s %s", key, value);
    if (ret <= 0) {
        printf(boardConfigDescription);
    } else if (ret == 2) {
        PAL_LOGI("Setting config value: %s = %s...\n", key, value);
        float val_f;
        uint32_t val_u32;
        char *dot = strchr(str, '.');
        if (dot == NULL) {
            ret = sscanf(value, "%ld", &val_u32);
            if (ret < 1 || config_set_value(key, &val_u32, 0) != STATUS_OK) {
                PAL_LOGE("Failed to set config value!\n");
            }
        } else {
            ret = sscanf(value, "%f", &val_f);
            if (ret < 1 || config_set_value(key, &val_f, 1) != STATUS_OK) {
                printf("ret %d\n", ret);
                PAL_LOGE("Failed to set config value!\n");
            }
        }
    }
}

const char *boardConfigDescription =
    "control_loop_period_ms: Period between state estimation updates (ms)\n"
    "sensor_loop_period_ms: Period between sensor reads without control "
    "requests (ms)\n"
    "storage_loop_period_ms: Period between file system flushes and pause "
    "checks (ms)\n"
    "gps_loop_period_ms: Period between GPS polls (ms)\n"
    "pspcom_rx_loop_period_ms: Period for checking incoming telemetry messages "
    "(ms)\n"
    "pspcom_tx_ground_loop_period_ms: Period for sending telemetry on the "
    "ground (ms)\n"
    "pspcom_tx_flight_loop_period_ms: Period for sending telemetry in flight "
    "(ms)\n"
    "\n"
    "STATE ESTIMATION SETTINGS\n"
    "state_init_time_ms: Duration to determine baseline sensor values (ms)\n"
    "boost_detect_period_ms: Time for accel to exceed threshold for launch "
    "detection (ms)\n"
    "launch_detect_replay: Flag for replaying state estimation during launch "
    "(1/0)\n"
    "min_fast_vel_mps: Minimum velocity considered 'fast' (m/s)\n"
    "min_boost_acc_mps2: Minimum acceleration for boost detection (m/s²)\n"
    "max_coast_acc_mps2: Maximum acceleration for coast detection (m/s²)\n"
    "max_grounded_alt_m: Maximum altitude to be considered grounded (m)\n"
    "min_grounded_time_ms: Minimum grounded time for transition (ms)\n"
    "max_ready_acc_bias_mps2: Maximum ready state acceleration bias (m/s²)\n"
    "\n"
    "STAGE SEPARATION SETTINGS\n"
    "stage_is_separator_bool: Stage separator status (1 = separate, 0 = "
    "don’t)\n"
    "stage_sep_lockout_ms: Lockout period post-launch for stage separation "
    "(ms)\n"
    "stage_sep_delay_ms: Delay after burnout for stage separation (ms)\n"
    "stage_min_sep_velocity_mps: Minimum separation velocity (m/s)\n"
    "stage_max_sep_velocity_mps: Maximum separation velocity (m/s)\n"
    "stage_min_sep_altitude_m: Minimum separation altitude (m)\n"
    "stage_max_sep_altitude_m: Maximum separation altitude (m)\n"
    "stage_min_sep_angle_deg: Minimum angle from vertical for separation (°)\n"
    "stage_max_sep_angle_deg: Maximum angle from vertical for separation (°)\n"
    "stage_sep_pyro_channel: Pyro channel for stage separation\n"
    "\n"
    "STAGE IGNITION SETTINGS\n"
    "stage_is_igniter_bool: Motor igniter status (1 = ignite, 0 = don’t)\n"
    "stage_ignite_lockout_ms: Lockout period post-launch for ignition (ms)\n"
    "stage_min_ignite_velocity_mps: Minimum velocity for ignition (m/s)\n"
    "stage_max_ignite_velocity_mps: Maximum velocity for ignition (m/s)\n"
    "stage_min_ignite_altitude_m: Minimum altitude for ignition (m)\n"
    "stage_max_ignite_altitude_m: Maximum altitude for ignition (m)\n"
    "stage_min_ignite_angle_deg: Minimum angle from vertical for ignition (°)\n"
    "stage_max_ignite_angle_deg: Maximum angle from vertical for ignition (°)\n"
    "stage_ignite_pyro_channel: Pyro channel for stage ignition\n"
    "\n"
    "RECOVERY SETTINGS\n"
    "main_height_m: Altitude for deploying the main parachute (m)\n"
    "drogue_delay_ms: Delay after apogee for drogue deployment (ms)\n"
    "deploy_lockout_ms: Lockout period after launch for pyro deployment (ms)\n"
    "\n"
    "TELEMETRY SETTINGS\n"
    "telemetry_frequency_hz: Frequency for telemetry transmission/reception "
    "(Hz)\n"
    "\n"
    "CHECKSUM\n"
    "checksum: CRC-32 checksum for config verification\n";

// Print firmware spec command
char regex_get_firmware_spec[] = "^get_firmware_spec[\n]*$";
void cmd_get_firmware_spec(char *str) {
    printf("Firmware spec: %s\n", FIRMWARE_SPECIFIER);
}

#endif  // COMMANDS_H