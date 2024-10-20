#ifndef BACKUP_H
#define BACKUP_H

#include <stdint.h>

#include "board.h"
#include "board_config.h"
#include "flight_control.h"
#include "gpio/gpio.h"
#include "state_estimation.h"
#include "stm32h735xx.h"

#define BKPSRAM_SIZE 4096  // bytes

typedef struct {
    uint64_t timestamp;
    uint32_t flag_mtp_pressed;
    BoardConfig board_config;
    StateEst state_estimate;
    FlightPhase flight_phase;
} Backup;

_Static_assert(sizeof(Backup) < BKPSRAM_SIZE, "Backup does not fit in BKPSRAM");

Status backup_init();

Backup* backup_get_ptr();

#endif  // BACKUP_H
