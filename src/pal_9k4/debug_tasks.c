#include "debug_tasks.h"

#include "FreeRTOS.h"
#include "malloc.h"
#include "stdio.h"
#include "task.h"

void debug_memory_usage_task() {
    while (1) {
        malloc_stats();

        vTaskDelay(5000);
    }
}