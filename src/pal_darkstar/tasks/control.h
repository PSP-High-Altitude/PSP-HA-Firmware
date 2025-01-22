#ifndef CONTROL_H
#define CONTROL_H

#include "sensor.pb.h"
#include "status.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

Status control_init();

Status control_update_sensors(const SensorFrame* sensor_frame);

void task_control(TaskHandle_t* handle_ptr);

#endif  // CONTROL_H
