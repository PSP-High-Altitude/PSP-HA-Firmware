#ifndef SENSORS_H
#define SENSORS_H

#include "iis2mdc/iis2mdc.h"
#include "kx134/kx134.h"
#include "lsm6dsox/lsm6dsox.h"
#include "ms5637/ms5637.h"
#include "status.h"

Status init_sensors();

#endif  // SENSORS_H
