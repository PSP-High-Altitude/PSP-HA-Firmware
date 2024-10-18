#ifndef PYROS_H
#define PYROS_H

#include <stdint.h>

#include "status.h"

// Max number of queued fire commands (including retries)
#define PYRO_QUEUE_LEN 8

// Pyro fire length (ms)
#define PYRO_FIRE_LENGTH_MS 1000

// Pyro retries (if we make it to the ground and the pyro didn't fire someone
// approaching could get injured)
#define PYRO_MAX_RETRIES 100

typedef enum {
    PYRO_MAIN,
    PYRO_DRG,
    PYRO_A1,
    PYRO_A2,
    PYRO_A3,
} Pyro;

Status pyros_init();

Status pyros_fire(Pyro pyro);

void task_pyros();

#endif