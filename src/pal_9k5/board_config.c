#include "board_config.h"

#include "backup.h"
#include "nand_flash.h"
#include "stdio.h"

static uint32_t s_valid_config_loaded = 0;

static FIL s_configfile;
static char s_configfile_path[] = CONFIG_DIR "/" CONFIG_FILENAME;

const static BoardConfig s_default_config = {
    .control_loop_period_ms = 10,             // ms
    .sensor_loop_period_ms = 100,             // ms
    .storage_loop_period_ms = 1000,           // ms
    .gps_loop_period_ms = 200,                // ms
    .pspcom_rx_loop_period_ms = 100,          // ms
    .pspcom_tx_ground_loop_period_ms = 5000,  // ms
    .pspcom_tx_flight_loop_period_ms = 1000,  // ms

    // State estimation settings
    .state_init_time_ms = 10000,     // ms
    .launch_detect_period_ms = 500,  // ms
    .launch_detect_replay = true,    // will replay
    .min_fast_vel_mps = 300,         // m/s
    .min_boost_acc_mps2 = 50,        // m/s^2
    .max_coast_acc_mps2 = 0,         // m/s^2

    // Stage separation settings
    .stage_is_separator_bool = 0,
    .stage_sep_delay_ms = 10000,
    .stage_min_sep_velocity_mps = 60.96f,
    .stage_max_sep_velocity_mps = 1e6f,
    .stage_min_sep_altitude_m = 100,
    .stage_max_sep_altitude_m = 1e6f,
    .stage_min_sep_angle_deg = 0,
    .stage_max_sep_angle_deg = 20,
    // Stage ignititon settings
    .stage_is_igniter_bool = 0,
    .stage_ignite_delay_ms = 10500,
    .stage_min_ignite_velocity_mps = 60.96f,
    .stage_max_ignite_velocity_mps = 1e6f,
    .stage_min_ignite_altitude_m = 100,
    .stage_max_ignite_altitude_m = 1e6f,
    .stage_min_ignite_angle_deg = 0,
    .stage_max_ignite_angle_deg = 0,

    // Recovery settings
    .main_height_m = 300.0,      // m
    .drogue_delay_ms = 1000,     // ms
    .deploy_lockout_ms = 10000,  // ms

    // Telemetry settings
    .telemetry_frequency_hz = 433000000,  // Hz
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

static Status load_config_from_flash(BoardConfig* config) {
    ASSERT_OK(nand_flash_open_file_for_read(&s_configfile, s_configfile_path),
              "failed to open config file\n");

    ASSERT_OK(nand_flash_read_data(&s_configfile, (uint8_t*)config,
                                   sizeof(BoardConfig)),
              "failed to read data from config file\n");

    EXPECT_OK(nand_flash_close_file(&s_configfile),
              "failed to close config file\n");

    return STATUS_OK;
}

BoardConfig* config_get_ptr() {
    if (s_valid_config_loaded) {
        return &(backup_get_ptr()->board_config);
    }

    return NULL;
}

// Ensure that a valid config is loaded into SRAM
Status config_load() {
    // Verify checksum of the config stored in the backup SRAM
    BoardConfig* sram_config = &(backup_get_ptr()->board_config);

    if (calc_config_checksum(sram_config) == sram_config->checksum) {
        // If the checksum verifies, do nothing
        s_valid_config_loaded = 1;
        PAL_LOGI("Config loaded from SRAM\n");
        config_print();
        return STATUS_OK;
    }

    // Otherwise, load the config from flash
    if (load_config_from_flash(sram_config) == STATUS_OK) {
        // Verify checksum of the config we just loaded
        if (calc_config_checksum(sram_config) == sram_config->checksum) {
            // If the checksum verifies, we're done
            s_valid_config_loaded = 2;
            PAL_LOGI("Config loaded from flash\n");
            config_print();
            return STATUS_OK;
        }
    }

    // If that didn't work either, load the default config
    *sram_config = s_default_config;
    sram_config->checksum = calc_config_checksum(sram_config);
    s_valid_config_loaded = 3;
    PAL_LOGI("Config loaded from defaults\n");

    config_print();

    return STATUS_OK;
}

// Save changes to the config by updating checksum and storing to flash
Status config_commit() {
    BoardConfig* sram_config = &(backup_get_ptr()->board_config);
    sram_config->checksum = calc_config_checksum(sram_config);

    ASSERT_OK(nand_flash_mkdir(CONFIG_DIR), "failed to create config dir\n");

    ASSERT_OK(nand_flash_open_file_for_write(&s_configfile, s_configfile_path),
              "failed to open config file\n");

    ASSERT_OK(nand_flash_write_data(&s_configfile, (uint8_t*)sram_config,
                                    sizeof(BoardConfig)),
              "failed to write data to config file\n");

    ASSERT_OK(nand_flash_close_file(&s_configfile),
              "failed to close config file\n");

    PAL_LOGI("Config committed to flash\n");

    s_valid_config_loaded = 2;

    return STATUS_OK;
}

// Force next load to load from NAND by invalidating SRAM copy
Status config_invalidate() {
    PAL_LOGW("Invalidating config\n");
    backup_get_ptr()->board_config.checksum ^= -1;
    s_valid_config_loaded = 0;

    return STATUS_OK;
}

void config_print() {
    BoardConfig* config = config_get_ptr();
    printf("===== BOARD CONFIG =====\n");
    printf("Control loop period: %lu ms\n", config->control_loop_period_ms);
    printf("Sensor loop period: %lu ms\n", config->sensor_loop_period_ms);
    printf("Storage loop period: %lu ms\n", config->storage_loop_period_ms);
    printf("GPS loop period: %lu ms\n", config->gps_loop_period_ms);
    printf("PSPCOM RX loop period: %lu ms\n", config->pspcom_rx_loop_period_ms);
    printf("PSPCOM TX ground loop period: %lu ms\n",
           config->pspcom_tx_ground_loop_period_ms);
    printf("PSPCOM TX flight loop period: %lu ms\n",
           config->pspcom_tx_flight_loop_period_ms);
    printf("State init time: %lu ms\n", config->state_init_time_ms);
    printf("Launch detect period: %lu ms\n", config->launch_detect_period_ms);
    printf("Launch detect replay: %lu\n", config->launch_detect_replay);
    printf("Min fast vel: %f m/s\n", config->min_fast_vel_mps);
    printf("Min boost acc: %f m/s^2\n", config->min_boost_acc_mps2);
    printf("Max coast acc: %f m/s^2\n", config->max_coast_acc_mps2);
    printf("Main height: %f m\n", config->main_height_m);
    printf("Drogue delay: %lu ms\n", config->drogue_delay_ms);
    printf("Deploy lockout: %lu ms\n", config->deploy_lockout_ms);
    printf("Telemetry frequency: %lu Hz\n", config->telemetry_frequency_hz);
    printf("Checksum: %08lx\n", config->checksum);
    printf("========================\n");
}