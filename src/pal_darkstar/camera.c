#include "camera.h"

#include "board.h"
#include "uart/uart.h"

static CameraMode s_camera_mode;

static UartDevice s_camera_device = {
    .baudrate = 115200,
    .tx = PIN_PC8,
    .rx = 0,
};

Status camera_init() {
    uart_init(&s_camera_device);

    // Assume the camera starts off
    s_camera_mode = CAMERA_MODE_OFF;

    return STATUS_OK;
}

Status camera_set_mode(CameraMode mode) {
    if (s_camera_mode == CAMERA_MODE_INV) {
        PAL_LOGE("Camera state is unknown; unable to set\n");
        return STATUS_ERROR;
    }

    // https://note.youdao.com/coshare/index.html?token=9AD3F89F0B92488E8241F58CAEDF7939&gid=29699666&_time=1740185861540
    // https://crccalc.com/?crc=cc0101&method=dvb-s2&datatype=hex&outtype=hex
    uint8_t cmd[] = {
        0xCC,  // Header (magic value)
        0x01,  // Command (camera control)
        0x01,  // Action ID (power button)
        0xE7,  // CRC-8/DVB-S2
    };

    if (mode == CAMERA_MODE_OFF && s_camera_mode == CAMERA_MODE_RUN) {
        uart_tx(&s_camera_device, cmd, sizeof(cmd));
        s_camera_mode = CAMERA_MODE_RUN;
    } else if (mode == CAMERA_MODE_RUN && s_camera_mode == CAMERA_MODE_OFF) {
        uart_tx(&s_camera_device, cmd, sizeof(cmd));
        s_camera_mode = CAMERA_MODE_OFF;
    }

    return STATUS_OK;
}

CameraMode camera_get_mode() {
    // Right now we don't have the capability to read status
    return s_camera_mode;
}
