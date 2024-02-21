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
    PYRO_AUX,
} Pyro;

Status init_pyros();

Status fire_pyro(Pyro pyro);

void pyros_task();

#endif