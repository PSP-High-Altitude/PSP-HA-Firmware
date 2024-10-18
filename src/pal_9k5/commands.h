#ifndef COMMANDS_H
#define COMMANDS_H

#include "board_config.h"
#include "regex.h"
#include "rtc/rtc.h"
#include "status.h"

// Help command
char regex_help[] = "^help[\n]*$";
void cmd_help(char *str) {
    printf(
        "Commands:\n"
        "  help                                  this command\n"
        "  set_datetime YYYY-MM-DD HH:MM:SS      sets the RTC time\n"
        "  get_datetime                          gets the RTC time\n"
        "  invalidate_config                     invalidates the config\n"
        "  set_frequency [frequency in Hz]       sets the frequency\n");
}

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

// Config invalidate command
char regex_invalidate_config[] = "^invalidate_config[\n]*$";
void cmd_invalidate_config(char *str) {
    config_invalidate();
    PAL_LOGW("Board config invalidated!\n");
}

// Config set frequency command
char regex_set_frequency[] = "^set_frequency [0-9]{1,10}[\n]*$";
void cmd_set_frequency(char *str) {
    uint32_t frequency;
    sscanf(str, "set_frequency %ld", &frequency);
    PAL_LOGW("Changed telemetry frequency to %.3f MHz!\n", frequency / 1e6);
}

#endif  // COMMANDS_H