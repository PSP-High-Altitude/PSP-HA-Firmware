#include "pyros.h"

#include <stdio.h>

Status pyros_init() { printf("Initialized pyros\n"); }

Status pyros_fire(Pyro pyro) { printf("Firing pyro %d\n", pyro); }
