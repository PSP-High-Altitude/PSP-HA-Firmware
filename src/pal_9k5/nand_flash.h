#ifndef NAND_FLASH_H
#define NAND_FLASH_H

#include "board_config.h"
#include "fatfs/ff.h"
#include "gps.pb.h"
#include "mt29f4g.h"
#include "sensor.pb.h"
#include "state.pb.h"

#define NAND_MOUNT_POINT "/NAND"

Status nand_flash_init();

Status nand_flash_reinit();

Status nand_flash_deinit();

Status nand_flash_flush(FIL* fp);

Status nand_flash_write_binary_data(FIL* fp, uint8_t* data, size_t size);

Status nand_flash_open_binary_file(FIL* fp, const char* fname);

char** nand_flash_get_file_list(const char* path, size_t* num_files);

Status nand_flash_delete_file_list(char** file_list, size_t num_files);

Status nand_flash_dump_prf_stats(char stats[]);

Status nand_flash_load_board_config(BoardConfig* board_config);

Status nand_flash_store_board_config(BoardConfig* board_config);

// Prints the files and directories at the specified path
// Returns an error code on failure
int f_ls(const char* path, int depth);

#endif  // NAND_FLASH_H