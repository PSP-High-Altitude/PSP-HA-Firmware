#include "rtc/rtc.h"

#include "stdio.h"
#include "stm32h7xx.h"

RTC_HandleTypeDef hrtc = {0};

Status rtc_init() {
    // Startup clocks
    HAL_PWR_EnableBkUpAccess();
    __NOP();
    __NOP();

    // Enable backup regulator
    HAL_PWREx_EnableBkUpReg();
    while (!(PWR->CR2 & PWR_CR2_BRRDY_Msk));

    // Enable battery charging
    HAL_PWREx_EnableBatteryCharging(PWR_BATTERY_CHARGING_RESISTOR_5);

    // If BDCR is in the incorrect state, reset it
    if ((RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_RTCCLKSOURCE_LSE) {
        printf("Backup domain in improper state, resetting\n");
        printf("%lx\n", RTC->ISR);
        __HAL_RCC_BACKUPRESET_FORCE();
        __HAL_RCC_BACKUPRESET_RELEASE();
        __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
    }

    // If RTC clock is not active enable it (RTC source bits must be written
    // first)
    if ((RCC->APB4ENR & RCC_APB4ENR_RTCAPBEN) == 0) {
        __HAL_RCC_RTC_CLK_ENABLE();
    }

    // If LSE is not active, enable it
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0) {
        __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);
        __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0) {
        }
    }

    if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0) {
        RCC->BDCR |= RCC_BDCR_RTCEN;
    }
    RTC_InitTypeDef rtc_init = {
        .AsynchPrediv = 127,
        .SynchPrediv = 255,
        .HourFormat = RTC_HOURFORMAT_24,
        .OutPut = RTC_OUTPUT_DISABLE,
        .OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH,
        .OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN,
        .OutPutRemap = RTC_OUTPUT_REMAP_NONE,
    };
    hrtc.Instance = RTC;
    hrtc.Init = rtc_init;
    if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0) {
        if (HAL_RTC_Init(&hrtc) != HAL_OK) {
            return STATUS_ERROR;
        }
    }
    return STATUS_OK;
}

Status rtc_deinit() {
    // Reset and set source
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
    __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
    return STATUS_OK;
}

RTCDateTime rtc_get_datetime() {
    RTCDateTime dt = {0};

    if (hrtc.State != HAL_RTC_STATE_READY) {
        return dt;
    }

    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;
    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

    dt.year = date.Year + 2000;
    dt.month = date.Month;
    dt.day = date.Date;

    dt.hour = time.Hours;
    dt.minute = time.Minutes;
    dt.second = time.Seconds;

    return dt;
}

void rtc_set_datetime(RTCDateTime dt) {
    RTC_DateTypeDef date = {0};
    date.Year = dt.year - 2000;
    date.Month = dt.month;
    date.Date = dt.day;

    RTC_TimeTypeDef time = {0};
    time.Hours = dt.hour;
    time.Minutes = dt.minute;
    time.Seconds = dt.second;

    HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
    HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
}

void rtc_print_datetime() {
    RTCDateTime dt = rtc_get_datetime();
    printf("%04ld-%02ld-%02ld %02ld:%02ld:%02ld\n", dt.year, dt.month, dt.day,
           dt.hour, dt.minute, dt.second);
}