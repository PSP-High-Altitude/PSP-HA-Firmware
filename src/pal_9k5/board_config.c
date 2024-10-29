#include "board_config.h"

#include "backup/backup.h"
#include "nand_flash.h"
#include "pyros.h"
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
    .max_grounded_alt_m = 100,       // m
    .min_grounded_time_ms = 10000,   // ms

    // Stage separation settings
    .stage_is_separator_bool = 0,
    .stage_sep_lockout_ms = 10000,
    .stage_min_sep_velocity_mps = -1e9f,
    .stage_max_sep_velocity_mps = 1e9f,
    .stage_min_sep_altitude_m = 100,
    .stage_max_sep_altitude_m = 1e9f,
    .stage_min_sep_angle_deg = -1e4,
    .stage_max_sep_angle_deg = 1e4,
    .stage_sep_pyro_channel = PYRO_A1,

    // Stage ignititon settings
    .stage_is_igniter_bool = 0,
    .stage_ignite_lockout_ms = 10500,
    .stage_min_ignite_velocity_mps = 60.96f,
    .stage_max_ignite_velocity_mps = 1e6f,
    .stage_min_ignite_altitude_m = 100,
    .stage_max_ignite_altitude_m = 1e6f,
    .stage_min_ignite_angle_deg = 0,
    .stage_max_ignite_angle_deg = 0,
    .stage_ignite_pyro_channel = PYRO_A1,

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

    printf("\n----- OS CONFIG -----\n");
    printf("Control loop period: %ld ms\n", config->control_loop_period_ms);
    printf("Sensor loop period: %ld ms\n", config->sensor_loop_period_ms);
    printf("Storage loop period: %ld ms\n", config->storage_loop_period_ms);
    printf("GPS loop period: %ld ms\n", config->gps_loop_period_ms);
    printf("PSPCOM RX loop period: %ld ms\n", config->pspcom_rx_loop_period_ms);
    printf("PSPCOM TX ground loop period: %ld ms\n",
           config->pspcom_tx_ground_loop_period_ms);
    printf("PSPCOM TX flight loop period: %ld ms\n",
           config->pspcom_tx_flight_loop_period_ms);

    printf("\n----- STATE ESTIMATION -----\n");
    printf("State init time: %ld ms\n", config->state_init_time_ms);
    printf("Launch detect period: %ld ms\n", config->launch_detect_period_ms);
    printf("Launch detect replay: %s\n",
           config->launch_detect_replay ? "Yes" : "No");
    printf("Min fast vel: %.2f m/s\n", config->min_fast_vel_mps);
    printf("Min boost acc: %.2f m/s^2\n", config->min_boost_acc_mps2);
    printf("Max coast acc: %.2f m/s^2\n", config->max_coast_acc_mps2);
    printf("Max grounded alt: %.2f m\n", config->max_grounded_alt_m);
    printf("Min grounded time: %.2f ms\n", config->min_grounded_time_ms);

    printf("\n----- STAGE SEPARATION -----\n");
    printf("Stage is separator: %s\n",
           config->stage_is_separator_bool ? "Yes" : "No");
    printf("Stage sep delay: %ld ms\n", config->stage_sep_lockout_ms);
    printf("Stage min sep vel: %.2f m/s\n", config->stage_min_sep_velocity_mps);
    printf("Stage max sep vel: %.2f m/s\n", config->stage_max_sep_velocity_mps);
    printf("Stage min sep alt: %.2f m\n", config->stage_min_sep_altitude_m);
    printf("Stage max sep alt: %.2f m\n", config->stage_max_sep_altitude_m);
    printf("Stage min sep angle: %.2f deg\n", config->stage_min_sep_angle_deg);
    printf("Stage max sep angle: %.2f deg\n", config->stage_max_sep_angle_deg);
    printf("Stage sep pyro channel: %ld\n", config->stage_sep_pyro_channel);

    printf("\n----- STAGE IGNITION -----\n");
    printf("Stage is igniter: %s\n",
           config->stage_is_igniter_bool ? "Yes" : "No");
    printf("Stage ignite delay: %ld ms\n", config->stage_ignite_lockout_ms);
    printf("Stage min ignite vel: %.2f m/s\n",
           config->stage_min_ignite_velocity_mps);
    printf("Stage max ignite vel: %.2f m/s\n",
           config->stage_max_ignite_velocity_mps);
    printf("Stage min ignite alt: %.2f m\n",
           config->stage_min_ignite_altitude_m);
    printf("Stage max ignite alt: %.2f m\n",
           config->stage_max_ignite_altitude_m);
    printf("Stage min ignite angle: %.2f deg\n",
           config->stage_min_ignite_angle_deg);
    printf("Stage max ignite angle: %.2f deg\n",
           config->stage_max_ignite_angle_deg);
    printf("Stage ignite pyro channel: %ld\n",
           config->stage_ignite_pyro_channel);

    printf("\n----- RECOVERY -----\n");
    printf("Main height: %.2f m\n", config->main_height_m);
    printf("Drogue delay: %ld ms\n", config->drogue_delay_ms);
    printf("Deploy lockout: %ld ms\n", config->deploy_lockout_ms);

    printf("\n----- TELEMETRY -----\n");
    printf("Telemetry frequency: %ld Hz\n", config->telemetry_frequency_hz);

    printf("\nChecksum: %08lx\n", config->checksum);
    printf("========================\n");
}

Status config_set_value(const char* key, void* value, int is_float) {
    BoardConfig* config = config_get_ptr();

    if (config == NULL) {
        return STATUS_ERROR;
    }

    float val_f;
    uint32_t val_u32;
    if (is_float) {
        val_f = *(float*)value;
        val_u32 = val_f;
    } else {
        val_u32 = *(uint32_t*)value;
        val_f = val_u32;
    }

    if (strcmp(key, "control_loop_period_ms") == 0) {
        config->control_loop_period_ms = val_u32;
    } else if (strcmp(key, "sensor_loop_period_ms") == 0) {
        config->sensor_loop_period_ms = val_u32;
    } else if (strcmp(key, "storage_loop_period_ms") == 0) {
        config->storage_loop_period_ms = val_u32;
    } else if (strcmp(key, "gps_loop_period_ms") == 0) {
        config->gps_loop_period_ms = val_u32;
    } else if (strcmp(key, "pspcom_rx_loop_period_ms") == 0) {
        config->pspcom_rx_loop_period_ms = val_u32;
    } else if (strcmp(key, "pspcom_tx_ground_loop_period_ms") == 0) {
        config->pspcom_tx_ground_loop_period_ms = val_u32;
    } else if (strcmp(key, "pspcom_tx_flight_loop_period_ms") == 0) {
        config->pspcom_tx_flight_loop_period_ms = val_u32;
    } else if (strcmp(key, "state_init_time_ms") == 0) {
        config->state_init_time_ms = val_u32;
    } else if (strcmp(key, "launch_detect_period_ms") == 0) {
        config->launch_detect_period_ms = val_u32;
    } else if (strcmp(key, "launch_detect_replay") == 0) {
        config->launch_detect_replay = val_u32;
    } else if (strcmp(key, "min_fast_vel_mps") == 0) {
        config->min_fast_vel_mps = val_f;
    } else if (strcmp(key, "min_boost_acc_mps2") == 0) {
        config->min_boost_acc_mps2 = val_f;
    } else if (strcmp(key, "max_coast_acc_mps2") == 0) {
        config->max_coast_acc_mps2 = val_f;
    } else if (strcmp(key, "max_grounded_alt_m") == 0) {
        config->max_grounded_alt_m = val_f;
    } else if (strcmp(key, "min_grounded_time_ms") == 0) {
        config->min_grounded_time_ms = val_u32;
    } else if (strcmp(key, "stage_is_separator_bool") == 0) {
        config->stage_is_separator_bool = val_u32;
    } else if (strcmp(key, "stage_sep_lockout_ms") == 0) {
        config->stage_sep_lockout_ms = val_u32;
    } else if (strcmp(key, "stage_min_sep_velocity_mps") == 0) {
        config->stage_min_sep_velocity_mps = val_f;
    } else if (strcmp(key, "stage_max_sep_velocity_mps") == 0) {
        config->stage_max_sep_velocity_mps = val_f;
    } else if (strcmp(key, "stage_min_sep_altitude_m") == 0) {
        config->stage_min_sep_altitude_m = val_f;
    } else if (strcmp(key, "stage_max_sep_altitude_m") == 0) {
        config->stage_max_sep_altitude_m = val_f;
    } else if (strcmp(key, "stage_min_sep_angle_deg") == 0) {
        config->stage_min_sep_angle_deg = val_f;
    } else if (strcmp(key, "stage_max_sep_angle_deg") == 0) {
        config->stage_max_sep_angle_deg = val_f;
    } else if (strcmp(key, "stage_sep_pyro_channel") == 0) {
        config->stage_sep_pyro_channel = val_u32;
    } else if (strcmp(key, "stage_is_igniter_bool") == 0) {
        config->stage_is_igniter_bool = val_u32;
    } else if (strcmp(key, "stage_ignite_lockout_ms") == 0) {
        config->stage_ignite_lockout_ms = val_u32;
    } else if (strcmp(key, "stage_min_ignite_velocity_mps") == 0) {
        config->stage_min_ignite_velocity_mps = val_f;
    } else if (strcmp(key, "stage_max_ignite_velocity_mps") == 0) {
        config->stage_max_ignite_velocity_mps = val_f;
    } else if (strcmp(key, "stage_min_ignite_altitude_m") == 0) {
        config->stage_min_ignite_altitude_m = val_f;
    } else if (strcmp(key, "stage_max_ignite_altitude_m") == 0) {
        config->stage_max_ignite_altitude_m = val_f;
    } else if (strcmp(key, "stage_min_ignite_angle_deg") == 0) {
        config->stage_min_ignite_angle_deg = val_f;
    } else if (strcmp(key, "stage_max_ignite_angle_deg") == 0) {
        config->stage_max_ignite_angle_deg = val_f;
    } else if (strcmp(key, "stage_ignite_pyro_channel") == 0) {
        config->stage_ignite_pyro_channel = val_u32;
    } else if (strcmp(key, "main_height_m") == 0) {
        printf("Setting main height to %.2f\n", val_f);
        config->main_height_m = val_f;
    } else if (strcmp(key, "drogue_delay_ms") == 0) {
        config->drogue_delay_ms = val_u32;
    } else if (strcmp(key, "deploy_lockout_ms") == 0) {
        config->deploy_lockout_ms = val_u32;
    } else if (strcmp(key, "telemetry_frequency_hz") == 0) {
        config->telemetry_frequency_hz = val_u32;
    } else {
        return STATUS_ERROR;
    }
    config_commit();
    return STATUS_OK;
}