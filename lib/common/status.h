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
    STATUS_TIMEOUT,
} Status;

#endif  // STATUS_H
