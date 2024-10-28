#ifndef STATUS_H
#define STATUS_H

typedef enum {
    STATUS_OK,
    STATUS_BUSY,
    STATUS_ERROR,
    STATUS_DATA_ERROR,
    STATUS_STATE_ERROR,
    STATUS_MEMORY_ERROR,
    STATUS_HARDWARE_ERROR,
    STATUS_TESTING_ERROR,
    STATUS_PARAMETER_ERROR,  // Error with invalid parameter being passed
    STATUS_TIMEOUT_ERROR,
} Status;

typedef enum {
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
} LogType;

#define STORAGE_FILENO 3
#define USB_FILENO 4

Status expect_ok(Status status, const char msg[], const char file[],
                 const int line);

void pal_log(LogType type, const char* format, ...);

// If the provided status is not OK, print out the error (evals to status)
#define EXPECT_OK(status, msg) (expect_ok((status), (msg), __FILE__, __LINE__))

// If the provided status is not OK, return from the current function
#define ASSERT_OK(status, msg)                        \
    do {                                              \
        Status retstatus = (status);                  \
        EXPECT_OK(retstatus, msg);                    \
        if (retstatus != STATUS_OK) return retstatus; \
    } while (0)

// Combine status codes
#define COMBINE_STATUS(status1, status2) \
    ((status1) > (status2) ? (status1) : (status2))

// Update status variable
#define UPDATE_STATUS(status, new_status)                             \
    do {                                                              \
        status = ((new_status) > (status) ? (new_status) : (status)); \
    } while (0)

#define PAL_LOGI(...) pal_log(LOG_INFO, __VA_ARGS__)
#define PAL_LOGW(...) pal_log(LOG_WARNING, __VA_ARGS__)
#define PAL_LOGE(...) pal_log(LOG_ERROR, __VA_ARGS__)

#endif  // STATUS_H
