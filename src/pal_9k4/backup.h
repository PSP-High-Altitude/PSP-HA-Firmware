#ifndef BACKUP_H
#define BACKUP_H

#include <stdint.h>

#include "board.h"
#include "flight_estimation.h"
#include "gpio/gpio.h"
#include "stm32h735xx.h"

#define BKPSRAM_SIZE 4096  // bytes

typedef struct {
    uint32_t flight_state_valid;
    FlightPhase flight_phase;
    StateEst state_est;
} Backup;

_Static_assert(sizeof(Backup) < BKPSRAM_SIZE, "Backup does not fit in BKPSRAM");

void init_backup();

Backup* get_backup_ptr();

#endif  // BACKUP_H
