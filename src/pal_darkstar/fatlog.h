#ifndef FATLOG_H
#define FATLOG_H

#include "board_config.h"
#include "fatfs/ff.h"
#include "gps.pb.h"
#include "sensor.pb.h"
#include "state.pb.h"

Status fatlog_init();

Status fatlog_reinit();

Status fatlog_deinit();

Status fatlog_flush(FIL* fp);

Status fatlog_read_data(FIL* fp, uint8_t* data, size_t size);

Status fatlog_write_data(FIL* fp, uint8_t* data, size_t size);

Status fatlog_open_file_for_write(FIL* fp, const char* fname);

Status fatlog_open_file_for_read(FIL* fp, const char* fname);

Status fatlog_close_file(FIL* fp);

Status fatlog_mkdir(const char* fname);

char** fatlog_get_file_list(const char* path, size_t* num_files);

Status fatlog_delete_file_list(char** file_list, size_t num_files);

Status fatlog_dump_prf_stats(char stats[]);

Status fatlog_space(uint64_t* total_bytes, uint64_t* free_bytes);

#endif  // FATLOG_H
