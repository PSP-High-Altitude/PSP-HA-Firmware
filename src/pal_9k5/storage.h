#ifndef STORAGE_H
#define STORAGE_H

#include "nand_flash.h"
#include "sd.h"
#include "status.h"

Status storage_init();

Status storage_write_log(const char *log, size_t size);

#endif  // STORAGE_H
