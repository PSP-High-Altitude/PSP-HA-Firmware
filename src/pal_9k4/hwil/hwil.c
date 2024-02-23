#include "hwil.h"

#include "timer.h"

static size_t s_hwil_sensor_data_idx = 0;
static size_t s_hwil_gps_data_idx = 0;

static GPS_Fix_TypeDef pb_frame_to_gps_fix(const GpsFrame* gps_frame) {
    GPS_Fix_TypeDef gps_fix;

    // Copy UTC Time
    gps_fix.year = gps_frame->year;
    gps_fix.month = gps_frame->month;
    gps_fix.day = gps_frame->day;
    gps_fix.hour = gps_frame->hour;
    gps_fix.min = gps_frame->min;
    gps_fix.sec = gps_frame->sec;

    // Unpack validity flags from uint64_t
    gps_fix.date_valid = (uint8_t)((gps_frame->valid_flags >> 0) & 0b1);
    gps_fix.time_valid = (uint8_t)((gps_frame->valid_flags >> 1) & 0b1);
    gps_fix.time_resolved = (uint8_t)((gps_frame->valid_flags >> 2) & 0b1);
    gps_fix.fix_type = (uint8_t)((gps_frame->valid_flags >> 3) & 0b111);
    gps_fix.fix_valid = (uint8_t)((gps_frame->valid_flags >> 8) & 0b1);
    gps_fix.diff_used = (uint8_t)((gps_frame->valid_flags >> 9) & 0b1);
    gps_fix.psm_state = (uint8_t)((gps_frame->valid_flags >> 10) & 0b111);
    gps_fix.hdg_veh_valid = (uint8_t)((gps_frame->valid_flags >> 14) & 0b1);
    gps_fix.carrier_phase = (uint16_t)((gps_frame->valid_flags >> 15) & 0b11);
    gps_fix.invalid_llh = (uint8_t)((gps_frame->valid_flags >> 19) & 0b1);

    // Copy Navigation info
    gps_fix.num_sats = gps_frame->num_sats;
    gps_fix.lon = gps_frame->lon;
    gps_fix.lat = gps_frame->lat;
    gps_fix.height = gps_frame->height;
    gps_fix.height_msl = gps_frame->height_msl;
    gps_fix.accuracy_horiz = gps_frame->accuracy_horiz;
    gps_fix.accuracy_vertical = gps_frame->accuracy_vertical;
    gps_fix.vel_north = gps_frame->vel_north;
    gps_fix.vel_east = gps_frame->vel_east;
    gps_fix.vel_down = gps_frame->vel_down;
    gps_fix.ground_speed = gps_frame->ground_speed;
    gps_fix.hdg = gps_frame->hdg;
    gps_fix.accuracy_speed = gps_frame->accuracy_speed;
    gps_fix.accuracy_hdg = gps_frame->accuracy_hdg;

    return gps_fix;
}

Status get_hwil_sensor_frame(SensorFrame* sensor_frame) {
    // Catch up the HWIL data stream to real time
    while ((HWIL_SENSOR_DATA[s_hwil_sensor_data_idx].timestamp < MICROS()) &&
           (s_hwil_sensor_data_idx < HWIL_SENSOR_DATA_SIZE)) {
        s_hwil_sensor_data_idx += 1;
    }

    // If we've run out of entries, abort
    if (s_hwil_sensor_data_idx >= HWIL_SENSOR_DATA_SIZE) {
        return STATUS_ERROR;
    }

    // Otherwise use the entry at the current index
    *sensor_frame = HWIL_SENSOR_DATA[s_hwil_sensor_data_idx];

    return STATUS_OK;
}

Status get_hwil_gps_fix(GPS_Fix_TypeDef* gps_fix) {
    // Catch up the HWIL data stream to real time
    while ((HWIL_GPS_DATA[s_hwil_gps_data_idx].timestamp < MICROS()) &&
           (s_hwil_gps_data_idx < HWIL_GPS_DATA_SIZE)) {
        s_hwil_gps_data_idx += 1;
    }

    // If we've run out of entries, abort
    if (s_hwil_gps_data_idx >= HWIL_GPS_DATA_SIZE) {
        return STATUS_ERROR;
    }

    // Otherwise use the entry at the current index
    *gps_fix = pb_frame_to_gps_fix(&HWIL_GPS_DATA[s_hwil_gps_data_idx]);

    return STATUS_OK;
}
