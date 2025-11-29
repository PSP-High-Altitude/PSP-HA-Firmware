#include "rtc/rtc.h"

#include <stdio.h>
#include <time.h>

// Retrieves the current date and time.
RTCDateTime rtc_get_datetime() {
    RTCDateTime dt;
    time_t t = time(NULL);
    if (t == ((time_t)-1)) {
        perror("Failed to obtain current time");
        // Initialize with zeroed values in case of error
        dt.year = dt.month = dt.day = 0;
        dt.hour = dt.minute = dt.second = 0;
        return dt;
    }

    struct tm *tm_info = localtime(&t);
    if (tm_info == NULL) {
        perror("Failed to convert time to local time");
        // Initialize with zeroed values in case of error
        dt.year = dt.month = dt.day = 0;
        dt.hour = dt.minute = dt.second = 0;
        return dt;
    }

    dt.year = tm_info->tm_year + 1900;
    dt.month = tm_info->tm_mon + 1;
    dt.day = tm_info->tm_mday;

    dt.hour = tm_info->tm_hour;
    dt.minute = tm_info->tm_min;
    dt.second = tm_info->tm_sec;

    return dt;
}

// Prints the current date and time in YYYY-MM-DD HH:MM:SS format.
void rtc_print_datetime() {
    RTCDateTime dt = rtc_get_datetime();
    if (dt.year == 0 && dt.month == 0 && dt.day == 0 && dt.hour == 0 &&
        dt.minute == 0 && dt.second == 0) {
        printf("Failed to retrieve current date and time.\n");
        return;
    }

    printf("Date and Time: %04u-%02u-%02u %02u:%02u:%02u\n", dt.year, dt.month,
           dt.day, dt.hour, dt.minute, dt.second);
}
