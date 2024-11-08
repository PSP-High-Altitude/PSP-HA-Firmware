#include "pyros.h"

#include <stdio.h>

#include "timer.h"

Status pyros_init() {
    printf("Initialized pyros\n");
    return STATUS_OK;
}

Status pyros_fire(Pyro pyro) {
    printf("Firing pyro %d @ %.3fs\n", pyro, MILLIS() / 1000.);
    return STATUS_OK;
}
