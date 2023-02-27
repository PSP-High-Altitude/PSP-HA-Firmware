#ifndef MAX_M10S_H
#define MAX_M10S_H

#include "i2c/i2c.h"
#include "status.h"

typedef enum {
    MAX_M10S_LAYER_GET_RAM = 0,
    MAX_M10S_LAYER_GET_BBR = 1,
    MAX_M10S_LAYER_GET_FLASH = 2,
    MAX_M10S_LAYER_GET_DEFAULT = 7,
    MAX_M10S_LAYER_SET_RAM = 1,
    MAX_M10S_LAYER_SET_BBR = 2,
    MAX_M10S_LAYER_SET_FLASH = 4,
    MAX_M10S_LAYER_SET_ALL = 7,
} Max_M10S_Layer_TypeDef;

typedef struct {
    // UTC Time
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;

    // Validity flags
    uint8_t date_valid;
    uint8_t time_valid;
    uint8_t time_resolved;

    /*
    0 = no fix, 1 = dead-reckoning only, 2 = 2D fix,
    3 = 3D fix, 4 = GNSS + dead-reckoning, 5 = time only
    */
    uint8_t fix_type;

    // Fix validity flags
    uint8_t fix_valid;
    uint8_t diff_used;
    /*
    0 = PSM not active, 1 = enabled, 2 = acquisition,
    3 = tracking, 4 = power optimized tracking, 5 = inactive
    */
    uint8_t psm_state;
    uint8_t hdg_veh_valid;
    /*
    0 = no carrier phase range soln, 1 = floating ambiguities,
    2 = fixed ambiguities
    */
    uint8_t carrier_phase;

    // Navigation info
    uint8_t num_sats;
    float lon;                // deg
    float lat;                // deg
    float height;             // m
    float height_msl;         // m
    float accuracy_horiz;     // m
    float accuracy_vertical;  // m
    float vel_north;          // m/s
    float vel_east;           // m/s
    float vel_down;           // m/s
    float ground_speed;       // m/s
    float hdg;                // deg
    float accuracy_speed;     // m/s
    float accuracy_hdg;       // deg

    // Additional flags
    uint8_t invalid_llh;  // Invalid lat, lon, height

} GPS_Fix_TypeDef;

Status max_m10s_init(I2cDevice* device);

Status max_m10s_poll_fix(I2cDevice* device, GPS_Fix_TypeDef* fix);

#endif