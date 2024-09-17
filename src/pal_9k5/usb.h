#ifndef USB_H
#define USB_H

#include <stdbool.h>
#include <stdint.h>

#include "status.h"

Status init_usb(bool mtp_mode);

extern int _write(int file, char *data, int len);

#endif  // USB_H
