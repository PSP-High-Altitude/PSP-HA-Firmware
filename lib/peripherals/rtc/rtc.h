#ifndef RTC_H
#define RTC_H

#include "status.h"
#include "stdint.h"

Status rtc_init();
Status rtc_deinit();

typedef struct {
    uint32_t year;
    uint32_t month;
    uint32_t day;

    uint32_t hour;
    uint32_t minute;
    uint32_t second;
} RTCDateTime;

RTCDateTime rtc_get_datetime();

void rtc_set_datetime(RTCDateTime dt);

void rtc_print_datetime();

#endif  // RTC_H