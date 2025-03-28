#ifndef CAMERA_H
#define CAMERA_H

#include "status.h"

typedef enum {
    CAMERA_MODE_INV,
    CAMERA_MODE_OFF,
    CAMERA_MODE_RUN,
} CameraMode;

Status camera_init();

Status camera_set_mode(CameraMode mode);
CameraMode camera_get_mode();

#endif  // CAMERA_H
