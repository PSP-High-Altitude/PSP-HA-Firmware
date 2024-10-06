#ifndef SENSORS_H
#define SENSORS_H

#include "status.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

Status init_sensors();

Status start_sensor_read();

void task_sensors(TaskHandle_t* handle_ptr);

#endif  // SENSORS_H
