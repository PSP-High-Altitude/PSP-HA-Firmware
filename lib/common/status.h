#ifndef STATUS_H
#define STATUS_H

typedef enum {
    OK,
    BUSY,
    ERROR,
    DATA_ERROR,
    HARDWARE_ERROR,
    TESTING_ERROR,
    PARAMETER_ERROR  // Error with invalid parameter being passed
} Status;

#endif  // STATUS_H
