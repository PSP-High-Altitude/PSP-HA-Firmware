#ifndef NAND_FLASH_H
#define NAND_FLASH_H

#include "board_config.h"
#include "fatfs/ff.h"
#include "gps.pb.h"
#include "mt29f4g.h"
#include "sensor.pb.h"
#include "state.pb.h"

#define NAND_MOUNT_POINT "/NAND"
#define NAND_LABEL "PAL 9000"

Status nand_flash_init();

Status nand_flash_reinit();

Status nand_flash_deinit();

Status nand_flash_flush(FIL* fp);

Status nand_flash_read_data(FIL* fp, uint8_t* data, size_t size);

Status nand_flash_write_data(FIL* fp, uint8_t* data, size_t size);

Status nand_flash_raw_write(const BYTE* buff, LBA_t sector, UINT count);

Status nand_flash_raw_read(BYTE* buff, LBA_t sector, UINT count);

Status nand_flash_open_file_for_write(FIL* fp, const char* fname);

Status nand_flash_open_file_for_read(FIL* fp, const char* fname);

Status nand_flash_close_file(FIL* fp);

Status nand_flash_mkdir(const char* fname);

char** nand_flash_get_file_list(const char* path, size_t* num_files);

Status nand_flash_delete_file_list(char** file_list, size_t num_files);

Status nand_flash_dump_prf_stats(char stats[]);

void nand_flash_capacity(uint32_t* block_count, uint16_t* block_size);

// Prints the files and directories at the specified path
// Returns an error code on failure
int f_ls(const char* path, int depth);

#endif  // NAND_FLASH_H