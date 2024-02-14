#ifndef STATUS_H
#define STATUS_H

typedef enum {
    STATUS_OK,
    STATUS_BUSY,
    STATUS_ERROR,
    STATUS_DATA_ERROR,
    STATUS_HARDWARE_ERROR,
    STATUS_TESTING_ERROR,
    STATUS_PARAMETER_ERROR,  // Error with invalid parameter being passed
    STATUS_TIMEOUT_ERROR,
} Status;

Status print_status_error(Status status, const char msg[], const char file[],
                          const int line);

#define PRINT_STATUS_ERROR(status, msg) \
    (print_status_error((status), (msg), __FILE__, __LINE__))

#endif  // STATUS_H
