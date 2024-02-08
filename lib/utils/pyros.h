#ifndef PYROS_H
#define PYROS_H

#include <stdint.h>
#define MAIN_PYRO 0
#define DROGUE_PYRO 1
#define AUX_PYRO(x) (2 + (x))

void init_pyros();

void fire_pyro(uint8_t pyro);

#endif