#ifndef NAND_FLASH_H
#define NAND_FLASH_H

#include "gps.pb.h"
#include "mt29f4g.h"
#include "sensor.pb.h"
#include "state.pb.h"

extern lfs_t g_fs;
extern struct lfs_config* g_lfs_cfg;

Status nand_flash_init();

Status nand_flash_reinit();

Status nand_flash_deinit();

Status nand_flash_flush();

Status nand_flash_write_sensor_data(SensorFrame* sensor_frame);

Status nand_flash_write_gps_data(GpsFrame* gps_frame);

Status nand_flash_write_state_data(StateFrame* state_frame);

Status nand_flash_dump_prf_stats(char stats[]);

// Prints the files and directories at the specified path
// Returns an error code on failure
int lfs_ls(lfs_t* lfs, const char* path);

#endif  // NAND_FLASH_H