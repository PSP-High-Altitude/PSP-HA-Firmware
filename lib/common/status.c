#include "status.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "rtc/rtc.h"

const char* STATUS_NAMES[] = {
    [STATUS_OK] = "STATUS_OK",
    [STATUS_BUSY] = "STATUS_BUSY",
    [STATUS_ERROR] = "STATUS_ERROR",
    [STATUS_DATA_ERROR] = "STATUS_DATA_ERROR",
    [STATUS_STATE_ERROR] = "STATUS_STATE_ERROR",
    [STATUS_MEMORY_ERROR] = "STATUS_MEMORY_ERROR",
    [STATUS_HARDWARE_ERROR] = "STATUS_HARDWARE_ERROR",
    [STATUS_TESTING_ERROR] = "STATUS_TESTING_ERROR",
    [STATUS_PARAMETER_ERROR] = "STATUS_PARAMETER_ERROR",
    [STATUS_TIMEOUT_ERROR] = "STATUS_TIMEOUT_ERROR",
};

extern int _write(int file, char* data, int len);

void fnoprintf(int file_no, const char* format, ...) {
    va_list args;
    va_start(args, format);
    int len = vsnprintf(NULL, 0, format, args);
    va_end(args);

    va_start(args, format);
    char* buf = malloc(len + 1);
    vsnprintf(buf, len + 1, format, args);
    _write(file_no, buf, len);
    free(buf);
    va_end(args);
}

Status expect_ok(Status status, const char msg[], const char file[],
                 const int line) {
    if (status != STATUS_OK) {
        RTCDateTime dt = rtc_get_datetime();
        fnoprintf(
            USB_FILENO,
            "\033[0;31m%04ld-%02ld-%02ld %02ld:%02ld:%02ld: ERROR at %s:%d: %s "
            "returned "
            "%s\n\033[0m",
            dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, file,
            line, msg, STATUS_NAMES[status]);
        fnoprintf(STORAGE_FILENO,
                  "%04ld-%02ld-%02ld %02ld:%02ld:%02ld: ERROR at %s:%d: %s "
                  "returned %s\n",
                  dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second,
                  file, line, msg, STATUS_NAMES[status]);
    }
    return status;
}

void pal_log(LogType type, const char* format, ...) {
    va_list args;
    va_start(args, format);

    RTCDateTime dt = rtc_get_datetime();
    switch (type) {
        case LOG_INFO:
            fnoprintf(USB_FILENO,
                      "\033[0;32m%04ld-%02ld-%02ld %02ld:%02ld:%02ld: INFO: ",
                      dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
            fnoprintf(STORAGE_FILENO,
                      "%04ld-%02ld-%02ld %02ld:%02ld:%02ld: INFO: ", dt.year,
                      dt.month, dt.day, dt.hour, dt.minute, dt.second);
            break;
        case LOG_WARNING:
            fnoprintf(USB_FILENO,
                      "\033[0;33m%04ld-%02ld-%02ld %02ld:%02ld:%02ld: WARN: ",
                      dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
            fnoprintf(STORAGE_FILENO,
                      "%04ld-%02ld-%02ld %02ld:%02ld:%02ld: WARN: ", dt.year,
                      dt.month, dt.day, dt.hour, dt.minute, dt.second);
            break;
        case LOG_ERROR:
            fnoprintf(USB_FILENO,
                      "\033[0;31m%04ld-%02ld-%02ld %02ld:%02ld:%02ld: ERR: ",
                      dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
            fnoprintf(STORAGE_FILENO,
                      "%04ld-%02ld-%02ld %02ld:%02ld:%02ld: ERR: ", dt.year,
                      dt.month, dt.day, dt.hour, dt.minute, dt.second);
            break;
    }

    vprintf(format, args);
    fnoprintf(USB_FILENO, "\033[0m");
    va_end(args);
}