#include "storage.h"

#include <sys/types.h>

#include "Regex.h"
#include "main.h"
#include "rtc.h"
#include "stdio.h"
#include "stdlib.h"
#include "timer.h"

#define DATA_DIR "/data"

static char s_logfile_path[64];

static FIL s_logfile;

Status storage_init() {
    // Initialize FATFS
    ASSERT_OK(diskio_init(NULL), "diskio init");
    ASSERT_OK(nand_flash_init(), "nand init");

    // Get a list of files in the data directory
    char** file_list = NULL;
    size_t num_files = 0;
    file_list = nand_flash_get_file_list(DATA_DIR, &num_files);

    // Get the current date for file names
    RTCDateTime dt = rtc_get_datetime();

    int max_num = 0;

    // Find the highest number in the NAND flash
    if (file_list != NULL) {
        // Check if any match the _YYYY-MM-DD-N+. pattern
        Regex regex;
        regexCompile(&regex, "_[0-9]{4}\\-[0-9]{2}\\-[0-9]{2}\\-[0-9]+\\.");

        for (size_t i = 0; i < num_files; i++) {
            Matcher match = regexMatch(&regex, file_list[i]);
            if (match.isFound) {
                // printf("Match: %.*s\n", (int)match.matchLength,
                //        file_list[i] + match.foundAtIndex);

                // Get all the values out of the match
                int year = atoi(file_list[i] + match.foundAtIndex + 1);
                int month = atoi(file_list[i] + match.foundAtIndex + 6);
                int day = atoi(file_list[i] + match.foundAtIndex + 9);
                int num = atoi(file_list[i] + match.foundAtIndex + 12);

                // Skip if the data doesn't match
                if (year != dt.year || month != dt.month || day != dt.day) {
                    continue;
                }

                if (num > max_num) {
                    max_num = num;
                }
            }
        }
    }

    // Free the file list
    nand_flash_delete_file_list(file_list, num_files);

    // Create the file paths
    sprintf(s_logfile_path, DATA_DIR "/log_%04ld-%02ld-%02ld-%d.txt", dt.year,
            dt.month, dt.day, max_num + 1);

    if (nand_flash_open_binary_file(&s_logfile, s_logfile_path) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

Status storage_write_log(const char* log, size_t size) {
    RTCDateTime dt = rtc_get_datetime();

    // Determine size of new buffer
    int new_size =
        snprintf(NULL, 0, "%04ld-%02ld-%02ld %02ld:%02ld:%02ld     %.*s",
                 dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, size,
                 log) +
        1;

    char* new_buf = malloc(new_size);

    snprintf(new_buf, new_size, "%04ld-%02ld-%02ld %02ld:%02ld:%02ld     %.*s",
             dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, size,
             log);

    Status status = nand_flash_write_binary_data(&s_logfile, (uint8_t*)new_buf,
                                                 new_size - 1);

    free(new_buf);
    return status;
}