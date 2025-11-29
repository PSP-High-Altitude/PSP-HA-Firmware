#ifndef PYROS_H
#define PYROS_H

#include <stdbool.h>
#include <stdint.h>

#include "status.h"

// Max number of queued fire commands (including retries)
// Should be greater than the number of pyro channels
#define PYRO_QUEUE_LEN (8)

// Pyro fire pulse width (ms)
#define PYRO_FIRE_LENGTH_MS (100)

// Pyro continuity check delay (ms)
#define PYRO_CHECK_DELAY_MS (1000)

// Pyro retries (if we make it to the ground and the pyro
// didn't fire someone approaching could get injured)
#define PYRO_MAX_RETRIES (10)

typedef enum {
    PYRO_MAIN,
    PYRO_DRG,
    PYRO_A1,
    PYRO_A2,
    PYRO_A3,
} Pyro;

Status pyros_init();

Status pyros_fire(Pyro pyro);
bool pyros_cont(Pyro pyro);

void task_pyros();

#endif