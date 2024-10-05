#include "board_config.h"

#include "backup.h"
#include "nand_flash.h"

static uint32_t s_valid_config_loaded = 0;

const static BoardConfig s_default_config = {
    .sampling_period_ms = 10,  // ms

    // State estimation settings
    .state_init_time_ms = 10000,     // ms
    .launch_detect_period_ms = 500,  // ms
    .launch_detect_replay = true,    // will replay
    .min_fast_vel_mps = 300,         // m/s
    .min_boost_acc_mps2 = 50,        // m/s^2
    .max_coast_acc_mps2 = 0,         // m/s^2

    // Recovery settings
    .main_height_m = 300.0,      // m
    .drogue_delay_ms = 1000,     // ms
    .deploy_lockout_ms = 10000,  // ms
};

// Simple summing checksum with non-zero initialization
static uint32_t calc_config_checksum(const BoardConfig* config) {
    const size_t size = sizeof(*config) - sizeof(config->checksum);
    const uint8_t* config_bytes = (const uint8_t*)config;

    uint32_t sum = 0x48414156;

    for (size_t i = 0; i < size; i++) {
        sum += (i + 1) * config_bytes[i];
    }

    return sum;
}

BoardConfig* get_config_ptr() {
    if (s_valid_config_loaded) {
        return &(get_backup_ptr()->board_config);
    }

    return NULL;
}

// Ensure that a valid config is loaded into SRAM
Status load_config() {
    // Verify checksum of the config stored in  the backup SRAM
    BoardConfig* sram_config = &(get_backup_ptr()->board_config);

    if (calc_config_checksum(sram_config) == sram_config->checksum) {
        // If the checksum verifies, do nothing
        s_valid_config_loaded = 1;
        return STATUS_OK;
    }

    // Otherwise, load the config from flash
    if (nand_flash_load_board_config(sram_config) == STATUS_OK) {
        // Verify checksum of the config we just loaded
        if (calc_config_checksum(sram_config) == sram_config->checksum) {
            // If the checksum verifies, we're done
            s_valid_config_loaded = 2;
            return STATUS_OK;
        }
    }

    // If that didn't work either, load the default config
    *sram_config = s_default_config;
    sram_config->checksum = calc_config_checksum(sram_config);
    s_valid_config_loaded = 3;

    return STATUS_OK;
}

// Save changes to the config by updating checksum and storing to flash
Status commit_config() {
    BoardConfig* sram_config = &(get_backup_ptr()->board_config);
    sram_config->checksum = calc_config_checksum(sram_config);

    ASSERT_OK(nand_flash_store_board_config(sram_config),
              "failed to store config to flash");

    s_valid_config_loaded = 2;

    return STATUS_OK;
}

// Force next load to load from NAND by invalidating SRAM copy
Status invalidate_config() {
    get_backup_ptr()->board_config.checksum ^= -1;
    s_valid_config_loaded = 0;

    return STATUS_OK;
}
