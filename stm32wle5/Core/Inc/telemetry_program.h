#ifndef TELEMETRY_PROGRAM_H
#define TELEMETRY_PROGRAM_H

#include "stdint.h"

void telemetry_program();

void process_packet_from_air(uint8_t *payload, uint16_t length);
void process_packet_from_uart();

#endif  // TELEMETRY_PROGRAM_H
