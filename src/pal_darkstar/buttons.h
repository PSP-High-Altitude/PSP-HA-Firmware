#ifndef BUTTONS_H
#define BUTTONS_H

#include "stddef.h"

void buttons_init();

void (*pause_event_callback)(void);
;

#endif  // BUTTONS_H