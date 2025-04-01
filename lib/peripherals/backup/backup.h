#ifndef BACKUP_H
#define BACKUP_H

#include <stdint.h>

#include "board_config.h"
#include "flight_control.h"
#include "gpio/gpio.h"
#include "state_estimation.h"

typedef struct {
    uint64_t timestamp;
    uint32_t flag_mtp_pressed;
    BoardConfig board_config;
    StateEst state_estimate;
    FlightPhase flight_phase;
    float ground_alt_m;
} Backup;

Status backup_init();

Backup* backup_get_ptr();

Status backup_invalidate();

#endif  // BACKUP_H
