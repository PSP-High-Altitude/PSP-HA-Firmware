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

Status expect_ok(Status status, const char msg[], const char file[],
                 const int line);

// If the provided status is not OK, print out the error (evals to status)
#define EXPECT_OK(status, msg) (expect_ok((status), (msg), __FILE__, __LINE__))

// If the provided status is not OK, return from the current function
#define ASSERT_OK(status, msg)                        \
    do {                                              \
        Status retstatus = (status);                  \
        EXPECT_OK(retstatus, msg);                    \
        if (retstatus != STATUS_OK) return retstatus; \
    } while (0)

#endif  // STATUS_H
