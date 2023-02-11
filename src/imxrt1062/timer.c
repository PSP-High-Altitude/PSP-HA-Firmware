#include "timer.h"

#include "imxrt1062/MIMXRT1062/drivers/fsl_clock.h"
#include "imxrt1062/MIMXRT1062/drivers/fsl_qtmr.h"

void initTimers() {
    CLOCK_SetXtalFreq(24000000);

    qtmr_config_t conf0 = {
        .primarySource = kQTMR_ClockDivide_1,
        .secondarySource = 0,
        .enableMasterMode = false,
        .enableExternalForce = false,
        .faultFilterCount = 0,
        .faultFilterPeriod = 0,
    };
    QTMR_Init(TMR1, kQTMR_Channel_0, &conf0);

    qtmr_config_t conf1 = {
        .primarySource = kQTMR_ClockCounter0Output,
        .secondarySource = 0,
        .enableMasterMode = false,
        .enableExternalForce = false,
        .faultFilterCount = 0,
        .faultFilterPeriod = 0,
    };
    QTMR_Init(TMR1, kQTMR_Channel_1, &conf1);

    qtmr_config_t conf2 = {
        .primarySource = kQTMR_ClockCounter1Output,
        .secondarySource = 0,
        .enableMasterMode = false,
        .enableExternalForce = false,
        .faultFilterCount = 0,
        .faultFilterPeriod = 0,
    };
    QTMR_Init(TMR1, kQTMR_Channel_2, &conf2);

    qtmr_config_t conf3 = {
        .primarySource = kQTMR_ClockCounter2Output,
        .secondarySource = 0,
        .enableMasterMode = false,
        .enableExternalForce = false,
        .faultFilterCount = 0,
        .faultFilterPeriod = 0,
    };
    QTMR_Init(TMR1, kQTMR_Channel_3, &conf3);

    QTMR_SetCompareValue(TMR1, kQTMR_Channel_0, (CLOCK_GetIpgFreq() / 1000000));
    QTMR_SetLoadValue(TMR1, kQTMR_Channel_0, 0);
    QTMR_SetCompareValue(TMR1, kQTMR_Channel_1, 0xFFFF);
    QTMR_SetLoadValue(TMR1, kQTMR_Channel_1, 0);
    QTMR_SetCompareValue(TMR1, kQTMR_Channel_2, 0xFFFF);
    QTMR_SetLoadValue(TMR1, kQTMR_Channel_2, 0);
    QTMR_SetCompareValue(TMR1, kQTMR_Channel_3, 0xFFFF);
    QTMR_SetLoadValue(TMR1, kQTMR_Channel_3, 0);

    QTMR_StartTimer(TMR1, kQTMR_Channel_3, kQTMR_CascadeCount);
    QTMR_StartTimer(TMR1, kQTMR_Channel_2, kQTMR_CascadeCount);
    QTMR_StartTimer(TMR1, kQTMR_Channel_1, kQTMR_CascadeCount);
    QTMR_StartTimer(TMR1, kQTMR_Channel_0, kQTMR_RisingEdge);

    for (uint8_t i = 0; i < 4; i++) {
        TMR1->CHANNEL[i].CNTR = 0U;
    }
}

void DELAY_NOP(uint16_t mS) {
    uint64_t numInstructions = (uint64_t)(mS / (1000.0 / 600000000 * 3));

    while (numInstructions > 0) {
        __NOP();
        numInstructions--;
    }
}

uint64_t MICROS() {
    uint64_t micros;
    micros = (uint64_t)QTMR_GetCurrentTimerCount(TMR1, kQTMR_Channel_3) << 32 |
             (uint64_t)QTMR_GetCurrentTimerCount(TMR1, kQTMR_Channel_2) << 16 |
             (uint64_t)QTMR_GetCurrentTimerCount(TMR1, kQTMR_Channel_1);
    return micros;
}

uint64_t MILLIS() { return MICROS() / 1000; }

void DELAY(uint16_t mS) {
    uint64_t start = MILLIS();
    while (MILLIS() < start + mS) {
    }
}

void DELAY_MICROS(uint32_t uS) {
    uint64_t start = MICROS();
    while (MICROS() < start + uS) {
    }
}