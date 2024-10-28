#include "buttons.h"

#include "FreeRTOS.h"
#include "backup.h"
#include "board_config.h"
#include "button_event.h"
#include "main.h"
#include "nand_flash.h"
#include "stdio.h"
#include "timer.h"
#include "timers.h"

static void mtp_button_handler();
static void pause_button_handler();

static ButtonEventConfig g_mtp_button = {
    .pin = PIN_MTP,
    .rising = true,
    .falling = false,
    .event_handler = mtp_button_handler,
};

static ButtonEventConfig g_pause_button = {
    .pin = PIN_PAUSE,
    .rising = true,
    .falling = true,
    .event_handler = pause_button_handler,
};

static xTimerHandle g_mtp_button_timer;

void pause_button_handler() {
    // Handle button press
    if (pause_event_callback != NULL) {
        pause_event_callback();
    }
}

// If the MTP button is not pressed
static void mtp_button_timeout(TimerHandle_t timer) {
    // Handle button timeout
    button_event_destroy(&g_mtp_button);
    PAL_LOGW("MTP mode was not selected!\n");

    // Transition to pause button
    button_event_create(&g_pause_button);
}

// If the MTP button is pressed
static void mtp_button_handler() {
    // Handle button press
    button_event_destroy(&g_mtp_button);
    xTimerStopFromISR(g_mtp_button_timer, 0);

    // Save the NAND flash
    nand_flash_deinit();

    DELAY_MICROS(1000000);
    backup_get_ptr()->flag_mtp_pressed = 1;
    config_invalidate();

    NVIC_SystemReset();
}

void buttons_init() {
    pause_event_callback = NULL;

    // Wait asynchronously to enter MTP mode
    if (!backup_get_ptr()->flag_mtp_pressed) {
        button_event_create(&g_mtp_button);
        g_mtp_button_timer =
            xTimerCreate("mtp_button_timeout", pdMS_TO_TICKS(5000), pdFALSE,
                         NULL, mtp_button_timeout);
        xTimerStart(g_mtp_button_timer, 0);
    }

    // Clear the flag so we go to normal mode next time
    backup_get_ptr()->flag_mtp_pressed = 0;
}
