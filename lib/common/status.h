#ifndef STATUS_H
#define STATUS_H

typedef enum {
    OK,
    BUSY,
    ERROR,
    DATA_ERROR,
    HARDWARE_ERROR,
    TESTING_ERROR,
} Status;

#endif  // STATUS_H
