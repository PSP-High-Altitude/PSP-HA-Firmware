#ifndef BUTTON_EVENT_H
#define BUTTON_EVENT_H

#include <stdint.h>

#include "status.h"

typedef struct {
    uint8_t pin;
    uint8_t rising;
    uint8_t falling;
    void (*event_handler)(void);
} ButtonEventConfig;

Status button_event_init();

Status button_event_create(ButtonEventConfig *config);

Status button_event_destroy(ButtonEventConfig *config);

#endif