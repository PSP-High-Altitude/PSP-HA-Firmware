#ifndef NAND_FLASH_H
#define NAND_FLASH_H

#include "gps.pb.h"
#include "mt29f4g.h"
#include "sensor.pb.h"
#include "state.pb.h"

#define NAND_ALLOW_REFORMAT

Status nand_flash_init();

Status nand_flash_reinit();

Status nand_flash_deinit();

Status nand_flash_flush();

Status nand_flash_write_sensor_data(SensorFrame* sensor_frame);

Status nand_flash_write_gps_data(GpsFrame* gps_frame);

Status nand_flash_write_state_data(StateFrame* state_frame);

Status nand_flash_dump_prf_stats(char stats[]);

#endif  // NAND_FLASH_H