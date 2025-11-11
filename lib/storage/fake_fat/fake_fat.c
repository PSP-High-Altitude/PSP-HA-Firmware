#include "fake_fat.h"

#include <memory.h>

const static uint8_t boot_sector[] = {
    //00    01    02    03    04    05    06    07    08    09    0A    0B    0C    0D    0E    0F
    0xEB, 0x58, 0x90,  'M',  'S',  'D',  'O',  'S',  '5',  '.',  '0', 0x00, 0x02, 0x04, 0x20, 0x00, // 00
    0x02, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, // 10
    0x00, 0x00, 0x20, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, // 20
    0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 30
    0x80, 0x00, 0x29, 0x12, 0x34, 0x56, 0x78,  'P',  'A',  'L',  ' ',  '9',  '0',  '0',  '0',  ' ', // 40
     ' ',  ' ',  'F',  'A',  'T',  '3',  '2',  ' ',  ' ',  ' ',                                     // 50
};

const static uint8_t signature[] = {0x55, 0xAA};

const static uint8_t fs_info_sector_start_signature[] = {0x52, 0x52, 0x61, 0x41};
const static uint8_t fs_info_sector_end[] = {0x72, 0x72, 0x41, 0x61, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00};

static void get_reserved_sector(uint32_t sector, uint8_t *buffer) {
    switch(sector) {
        case 0:
        case 6:
            memcpy(buffer, boot_sector, sizeof(boot_sector));
            memcpy(buffer + 0x1FE, signature, sizeof(signature));
            break;
        case 1:
        case 7:
            memcpy(buffer, fs_info_sector_start_signature, sizeof(fs_info_sector_start_signature));
            memcpy(buffer + 0x1E4, fs_info_sector_end, sizeof(fs_info_sector_end));
            memcpy(buffer + 0x1FE, signature, sizeof(signature));
            break;
        case 3:
        case 8:
            memcpy(buffer + 0x1FE, signature, sizeof(signature));
            break;
        default:
            break;
    }
}

static void get_fat_table_sector(uint32_t sector, uint8_t *buffer) {
    if (sector == 0x20 || sector == 0x1020) {
        // Media descriptor and end-of-chain markers
        buffer[0] = 0xF8;
        buffer[1] = 0xFF;
        buffer[2] = 0xFF;
        buffer[3] = 0x0F;

        buffer[4] = 0xFF;
        buffer[5] = 0xFF;
        buffer[6] = 0xFF;
        buffer[7] = 0xFF;

        buffer[8] = 0xFF;
        buffer[9] = 0xFF;
        buffer[10] = 0xFF;
        buffer[11] = 0x0F;

        buffer[12] = 0xFF;
        buffer[13] = 0xFF;
        buffer[14] = 0xFF;
        buffer[15] = 0x0F;

        buffer[16] = 0xFF;
        buffer[17] = 0xFF;
        buffer[18] = 0xFF;
        buffer[19] = 0x0F;

        buffer[20] = 0xFF;
        buffer[21] = 0xFF;
        buffer[22] = 0xFF;
        buffer[23] = 0x0F;
    }
}

static void get_fat_data_sector(uint32_t sector, uint8_t *buffer) {
    static const char test_file_content[] = "This is a test file for the fake FAT filesystem on PAL!\n";

    if (sector == 0x2020) {
        fake_fat_dirent_t volume_label_entry = {
            .name = "PAL 9000",
            .ext = "   ",
            .attr = 0x08,  // Volume label
            .flags_reserved = 0,
            .creation_time_hundredths = 0,
            .creation_time = 0,
            .creation_date = 0,
            .last_access_date = 0,
            .first_cluster_high = 0,
            .last_mod_time = 0,
            .last_mod_date = 0,
            .first_cluster_low = 0,
            .size = 0
        };
        fake_fat_dirent_t test_file_entry = {
            .name = "TEST    ",
            .ext = "TXT",
            .attr = 0x20,  // Archive
            .flags_reserved = 0x18,
            .creation_time_hundredths = 0,
            .creation_time = 0,
            .creation_date = 0x21,
            .last_access_date = 0x21,
            .first_cluster_high = 0,
            .last_mod_time = 0,
            .last_mod_date = 0x21,
            .first_cluster_low = 5,
            .size = sizeof(test_file_content) - 1
        };

        memcpy(buffer, &volume_label_entry, sizeof(volume_label_entry));
        memcpy(buffer + sizeof(fake_fat_dirent_t), &test_file_entry, sizeof(test_file_entry));
    }

    else if (sector == 0x202C) {
        memcpy(buffer, test_file_content, sizeof(test_file_content) - 1);
    }
}

Status fake_fat_init() {
    return STATUS_OK;
}


void fake_fat_deinit() {

}

uint32_t fake_fat_get_total_sectors() {
    return 0x200000;
}

void fake_fat_read(uint32_t sector,
                   uint8_t *buffer) {

    memset(buffer, 0, 512);

    // Reserved sectors: 0x00 - 0x1F
    if (sector < 0x20) {
        get_reserved_sector(sector, buffer);
        return;
    }

    // FAT table area: 0x20 - 0x201F
    if (sector >= 0x20 && sector < 0x2020) {
        get_fat_table_sector(sector, buffer);
        return;
    }

    // FAT data area: 0x2020+
    if (sector >= 0x2020) {
        get_fat_data_sector(sector, buffer);
        return;
    }
    
}