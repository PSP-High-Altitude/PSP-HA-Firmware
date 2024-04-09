/*
 * YAFFS: Yet another Flash File System . A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2018 Aleph One Ltd.
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1 as
 * published by the Free Software Foundation.
 *
 * Note: Only YAFFS headers are LGPL, YAFFS C code is covered by GPL.
 */

/*
 * Header file for using yaffs in an application via
 * a direct interface.
 */

#ifndef __YAFFS_OSGLUE_H__
#define __YAFFS_OSGLUE_H__

#include "FreeRTOS.h"
#include "main.h"
#include "semphr.h"
#include "yaffs2/yaffs_trace.h"
#include "yaffs2/yportenv.h"

// #define YAFFS_DEBUG_HIGH

SemaphoreHandle_t yaffsfs_LockSemaphore;
int yaffs_err;

#if defined(DEBUG) && defined(YAFFS_DEBUG_HIGH)
unsigned yaffs_trace_mask =

    YAFFS_TRACE_SCAN | YAFFS_TRACE_GC | YAFFS_TRACE_ERASE | YAFFS_TRACE_ERROR |
    YAFFS_TRACE_TRACING | YAFFS_TRACE_ALLOCATE | YAFFS_TRACE_BAD_BLOCKS |
    YAFFS_TRACE_VERIFY | 0;
#elif defined(DEBUG)
unsigned yaffs_trace_mask = YAFFS_TRACE_ERROR | YAFFS_TRACE_BAD_BLOCKS | 0;
#else
unsigned yaffs_trace_mask = 0;
#endif

void yaffs_bug_fn(const char *file_name, int line_no) {}

void yaffsfs_Lock(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
        !xPortIsInsideInterrupt())
        xSemaphoreTake(yaffsfs_LockSemaphore, 100);
}

void yaffsfs_Unlock(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
        !xPortIsInsideInterrupt())
        xSemaphoreGive(yaffsfs_LockSemaphore);
}

u32 yaffsfs_CurrentTime(void) { return 0; }

void yaffsfs_SetError(int err) { yaffs_err = err; }

void *yaffsfs_malloc(size_t size) { return malloc(size); }
void yaffsfs_free(void *ptr) { free(ptr); }

void yaffsfs_get_malloc_values(unsigned *current, unsigned *high_water) {}

int yaffsfs_CheckMemRegion(const void *addr, size_t size, int write_request) {
    return 0;
}

void yaffsfs_OSInitialisation(void) {
    yaffsfs_LockSemaphore = xSemaphoreCreateMutex();
}

#endif
