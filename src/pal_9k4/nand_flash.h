#ifndef NAND_FLASH_H
#define NAND_FLASH_H

#include "gps.pb.h"
#include "mt29f4g.h"
#include "sensor.pb.h"
#include "state.pb.h"

#define NAND_MOUNT_POINT "/NAND"

Status nand_flash_init();

Status nand_flash_reinit();

Status nand_flash_deinit();

Status nand_flash_flush();

Status nand_flash_write_sensor_data(pb_byte_t* sensor_frame, size_t size);

Status nand_flash_write_gps_data(pb_byte_t* gps_frame, size_t size);

Status nand_flash_write_state_data(pb_byte_t* state_frame, size_t size);

Status nand_flash_dump_prf_stats(char stats[]);

// Prints the files and directories at the specified path
// Returns an error code on failure
int f_ls(const char* path);

#endif  // NAND_FLASH_H