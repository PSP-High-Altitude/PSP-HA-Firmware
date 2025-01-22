#ifndef SENSORS_H
#define SENSORS_H

#include "status.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

Status sensors_init();

Status sensors_start_read();

void task_sensors(TaskHandle_t* handle_ptr);

#endif  // SENSORS_H
