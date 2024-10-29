#ifndef COMMANDS_H
#define COMMANDS_H

#include "board_config.h"
#include "mt29f4g.h"
#include "pspcom.h"
#include "regex.h"
#include "rtc/rtc.h"
#include "status.h"

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
        "  erase_flash_chip                      full block-level flash erase\n"
        "  set_frequency [frequency in Hz]       sets the frequency\n"
        "  set_config_value [key] [value]        sets a config value\n");
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
char regex_set_config_value[] = "^set_config_value.*$";
void cmd_set_config_value(char *str) {
    char key[256];
    char value[256];
    int ret = sscanf(str, "set_config_value %s %s", key, value);
    if (ret <= 0) {
        printf(
            "================== Configuration Options ==================\n"
            "       period in ms between state estimation update steps\n"
            "       uint32_t control_loop_period_ms;\n"
            "       period in ms between sensor reads in the absence of "
            "control requests\n"
            "       uint32_t sensor_loop_period_ms;\n"
            "       period in ms between file system flushes and pause request "
            "checks\n"
            "       uint32_t storage_loop_period_ms;\n"
            "       period in ms between polling the GPS\n"
            "       uint32_t gps_loop_period_ms;\n"
            "       period in ms between checking for incoming telemetry "
            "messages\n"
            "       uint32_t pspcom_rx_loop_period_ms;\n"
            "       period in ms between sending standard telemetry message on "
            "the ground\n"
            "       uint32_t pspcom_tx_ground_loop_period_ms;\n"
            "       period in ms between sending standard telemetry message in "
            "flight\n"
            "       uint32_t pspcom_tx_flight_loop_period_ms;\n"
            "       time in ms during which a baseline value for the sensors "
            "is determined\n"
            "       uint32_t state_init_time_ms;\n"
            "       time in ms for which accel must be above boost threshold "
            "to detect launch\n"
            "       uint32_t launch_detect_period_ms;\n"
            "       whether to replay state estimation of launch detection "
            "interval at exit\n"
            "       uint32_t launch_detect_replay;\n"
            "       vertical velocity over which we're considered to be fast\n"
            "       float min_fast_vel_mps;\n"
            "       launch minimum acceleration above which we are considered "
            "to be in boost\n"
            "       float min_boost_acc_mps2;\n"
            "       maximum acceleration below which we are considered to be "
            "in coast\n"
            "       float max_coast_acc_mps2;\n"
            "       maximum altitude under which we are considered to be "
            "grounded\n"
            "       float max_grounded_alt_m;\n"
            "       minimum time for which we have to be below the grounded "
            "alt to transition\n"
            "       float min_grounded_time_ms;\n"
            "       \n"
            "       /* STAGE SEPARATION SETTINGS */\n"
            "       status of this board as a stage separator\n"
            "       (1 to fire separation charge, 0 to not)\n"
            "       uint8_t stage_is_separator_bool;\n"
            "       lockout after launch in ms to separate stages (NOT TO "
            "IGNITE)\n"
            "       uint32_t stage_sep_lockout_ms;\n"
            "       minimum velocity in m/s to separate stages\n"
            "       float stage_min_sep_velocity_mps;\n"
            "       maximum velocity in m/s to separate stages\n"
            "       float stage_max_sep_velocity_mps;\n"
            "       minimum altitude in m to separate stages\n"
            "       float stage_min_sep_altitude_m;\n"
            "       maximum altitude in m to separate stages\n"
            "       float stage_max_sep_altitude_m;\n"
            "       minimum angle from vertical in deg to separate stages\n"
            "       float stage_min_sep_angle_deg;\n"
            "       maximum angle from vertical in deg to separate stages\n"
            "       float stage_max_sep_angle_deg;\n"
            "       stage separation pyro channel\n"
            "       uint32_t stage_sep_pyro_channel;\n"
            "       \n"
            "       /* STAGE IGNITION SETTINGS */\n"
            "       status of this board as a MOTOR IGNITER\n"
            "       (1 to IGNITE motor after separation, 0 to NOT IGNITE)\n"
            "       uint8_t stage_is_igniter_bool;\n"
            "       lockout after launch in ms to IGNITE the next stage\n"
            "       uint32_t stage_ignite_lockout_ms;\n"
            "       minimum velocity in m/s to IGNITE stage\n"
            "       float stage_min_ignite_velocity_mps;\n"
            "       maximum velocity in m/s to IGNITE stage\n"
            "       float stage_max_ignite_velocity_mps;\n"
            "       minimum altitude in m to IGNITE stage\n"
            "       float stage_min_ignite_altitude_m;\n"
            "       maximum altitude in m to IGNITE stage\n"
            "       float stage_max_ignite_altitude_m;\n"
            "       minimum angle from vertical in deg to IGNITE stage\n"
            "       float stage_min_ignite_angle_deg;\n"
            "       maximum angle from vertical in deg to IGNITE stage\n"
            "       float stage_max_ignite_angle_deg;\n"
            "       stage separation pyro channel\n"
            "       uint32_t stage_ignite_pyro_channel;\n"
            "       \n"
            "       /* RECOVERY SETTINGS */\n"
            "       height above ground in m at which main pyro is fired\n"
            "       float main_height_m;\n"
            "       delay in ms from detecting apogee to firing drogue pyro\n"
            "       uint32_t drogue_delay_ms;\n"
            "       time in ms from launch detection during which pyros cannot "
            "fire\n"
            "       uint32_t deploy_lockout_ms;\n"
            "       \n"
            "       /* TELEMETRY SETTINGS */\n"
            "       Radio frequency in Hz at which telemetry is sent and "
            "received\n"
            "       uint32_t telemetry_frequency_hz;\n"
            "       CRC-32 checksum of the config\n"
            "       uint32_t checksum;");
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

#endif  // COMMANDS_H