#ifndef GPS_H
#define GPS_H

#include "status.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

Status gps_init();

void task_gps(TaskHandle_t* handle_ptr);

#endif  // GPS_H
