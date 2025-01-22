#ifndef VOLTAGE_H
#define VOLTAGE_H

#include "status.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// Initialize the voltage subsystem.
Status voltage_init();

// Voltage task: measures all available voltages.
void task_voltage(TaskHandle_t* handle_ptr);

#endif  // VOLTAGE_H