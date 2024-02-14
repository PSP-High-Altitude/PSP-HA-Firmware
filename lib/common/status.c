#include "status.h"

#include <stdio.h>

const char* STATUS_NAMES[] = {
    [STATUS_OK] = "STATUS_OK",
    [STATUS_BUSY] = "STATUS_BUSY",
    [STATUS_ERROR] = "STATUS_ERROR",
    [STATUS_DATA_ERROR] = "STATUS_DATA_ERROR",
    [STATUS_HARDWARE_ERROR] = "STATUS_HARDWARE_ERROR",
    [STATUS_TESTING_ERROR] = "STATUS_TESTING_ERROR",
    [STATUS_PARAMETER_ERROR] = "STATUS_PARAMETER_ERROR",
    [STATUS_TIMEOUT_ERROR] = "STATUS_TIMEOUT_ERROR",
};

Status print_status_error(Status status, const char msg[]) {
    if (status != STATUS_OK) {
        printf("ERROR at %s:%d: %s returned %s", __FILE__, __LINE__, msg,
               STATUS_NAMES[status]);
    }
    return status;
}
