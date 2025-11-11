#ifndef FAKE_FAT_H
#define FAKE_FAT_H

#include "status.h"

typedef struct fake_fat_dirent {
    char name[8];
    char ext[3];
    uint8_t attr;
    uint8_t flags_reserved;
    uint8_t creation_time_hundredths;
    uint16_t creation_time;
    uint16_t creation_date;
    uint16_t last_access_date;
    uint16_t first_cluster_high;
    uint16_t last_mod_time;
    uint16_t last_mod_date;
    uint16_t first_cluster_low;
    uint32_t size;
} fake_fat_dirent_t;

Status fake_fat_init();
void fake_fat_deinit();

uint32_t fake_fat_get_total_sectors();

// Read a single sector
void fake_fat_read(uint32_t sector,
                   uint8_t *buffer);

#endif // FAKE_FAT_H