#ifndef USB_H
#define USB_H

#include <stdbool.h>
#include <stdint.h>

#include "status.h"

#define SERIAL_BUFFER_SIZE 256

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

Status init_usb(bool mtp_mode);

extern int _write(int file, char *data, int len);

#endif  // USB_H
