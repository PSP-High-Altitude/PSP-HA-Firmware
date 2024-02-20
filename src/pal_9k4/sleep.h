#ifndef SLEEP_H
#define SLEEP_H

#include <stdint.h>

#define USE_DEEPSLEEP

extern uint64_t g_last_tickless_idle_entry_us;
extern uint64_t g_total_tickless_idle_us;

void pre_sleep(uint32_t* xModifiableIdleTime);
void post_sleep(uint32_t* xExpectedIdleTime);

#endif  // SLEEP_H
