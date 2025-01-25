/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::File System
 * Copyright (c) 2004-2024 Arm Limited (or its affiliates). All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    fs_rtos2.h
 * Purpose: File System RTOS abstraction for CMSIS-RTOS2
 *----------------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "fs_core.h"
#include "semphr.h"

/*
  Create and initialize a mutex object
*/
FS_MUTEX fs_mutex_new(const void *arg) {
    return ((FS_MUTEX)xSemaphoreCreateMutex());
}
/*
  Acquire a mutex.
*/
uint32_t fs_mutex_acquire(FS_MUTEX mutex) {
    uint32_t status = 0U;

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        status = 1U;
    }
    return (status);
}
/*
  Release a mutex.
*/
uint32_t fs_mutex_release(FS_MUTEX mutex) {
    uint32_t status = 0U;

    if (xSemaphoreGive(mutex) != pdTRUE) {
        status = 1U;
    }
    return (status);
}
/*
  Delete a mutex object.
*/
uint32_t fs_mutex_delete(FS_MUTEX mutex) {
    uint32_t status = 0U;

    vSemaphoreDelete(mutex);
    return (status);
}
/*
  Get the RTOS kernel tick frequency
*/
uint32_t fs_get_rtos_tick_freq(void) { return 1000U; }
/*
  Number of RTOS ticks in a millisecond
*/
uint32_t fs_ms_rtos_tick;
/*
  Wait for Timeout
*/
uint32_t fs_set_rtos_delay(uint32_t millisec) {
    vTaskDelay(pdMS_TO_TICKS(millisec));
    return (0);
}
/*
  Get the RTOS kernel system timer count.
*/
uint32_t fs_get_sys_tick(void) { return (xTaskGetTickCount()); }
/*
  Convert a microseconds value to a RTOS kernel system timer value.
*/
uint32_t fs_get_sys_tick_us(uint32_t microsec) {
    return (pdMS_TO_TICKS(microsec / 1000U));
}
