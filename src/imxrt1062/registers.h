#pragma once

#include <stdint.h>

//  FlexIO Registers

#define FLEXIO1_ADDR 0x401AC000u
#define FLEXIO2_ADDR 0x401B0000u
#define FLEXIO3_ADDR 0x42020000u

typedef struct {
    volatile uint32_t VERID;
    volatile uint32_t PARAM;
    volatile uint32_t CTRL;
    volatile uint32_t PIN;
    volatile uint32_t SHIFTSTAT;
    volatile uint32_t SHIFTERR;
    volatile uint32_t TIMSTAT;
    uint8_t RESERVED_0[4];
    volatile uint32_t SHIFTSIEN;
    volatile uint32_t SHIFTEIEN;
    volatile uint32_t TIMIEN;
    uint8_t RESERVED_1[4];
    volatile uint32_t SHIFTSDEN;
    uint8_t RESERVED_2[12];
    volatile uint32_t SHIFTSTATE;
    uint8_t RESERVED_3[60];
    volatile uint32_t SHIFTCTL0;
    volatile uint32_t SHIFTCTL1;
    volatile uint32_t SHIFTCTL2;
    volatile uint32_t SHIFTCTL3;
    volatile uint32_t SHIFTCTL4;
    volatile uint32_t SHIFTCTL5;
    volatile uint32_t SHIFTCTL6;
    volatile uint32_t SHIFTCTL7;
    uint8_t RESERVED_4[96];
    volatile uint32_t SHIFTCFG0;
    volatile uint32_t SHIFTCFG1;
    volatile uint32_t SHIFTCFG2;
    volatile uint32_t SHIFTCFG3;
    volatile uint32_t SHIFTCFG4;
    volatile uint32_t SHIFTCFG5;
    volatile uint32_t SHIFTCFG6;
    volatile uint32_t SHIFTCFG7;
    uint8_t RESERVED_5[224];
    volatile uint32_t SHIFTBUF0;
    volatile uint32_t SHIFTBUF1;
    volatile uint32_t SHIFTBUF2;
    volatile uint32_t SHIFTBUF3;
    volatile uint32_t SHIFTBUF4;
    volatile uint32_t SHIFTBUF5;
    volatile uint32_t SHIFTBUF6;
    volatile uint32_t SHIFTBUF7;
    uint8_t RESERVED_6[96];
    volatile uint32_t SHIFTBUFBIS0;
    volatile uint32_t SHIFTBUFBIS1;
    volatile uint32_t SHIFTBUFBIS2;
    volatile uint32_t SHIFTBUFBIS3;
    volatile uint32_t SHIFTBUFBIS4;
    volatile uint32_t SHIFTBUFBIS5;
    volatile uint32_t SHIFTBUFBIS6;
    volatile uint32_t SHIFTBUFBIS7;
    uint8_t RESERVED_7[96];
    volatile uint32_t SHIFTBUFBYS0;
    volatile uint32_t SHIFTBUFBYS1;
    volatile uint32_t SHIFTBUFBYS2;
    volatile uint32_t SHIFTBUFBYS3;
    volatile uint32_t SHIFTBUFBYS4;
    volatile uint32_t SHIFTBUFBYS5;
    volatile uint32_t SHIFTBUFBYS6;
    volatile uint32_t SHIFTBUFBYS7;
    uint8_t RESERVED_8[96];
    volatile uint32_t SHIFTBUFBBS0;
    volatile uint32_t SHIFTBUFBBS1;
    volatile uint32_t SHIFTBUFBBS2;
    volatile uint32_t SHIFTBUFBBS3;
    volatile uint32_t SHIFTBUFBBS4;
    volatile uint32_t SHIFTBUFBBS5;
    volatile uint32_t SHIFTBUFBBS6;
    volatile uint32_t SHIFTBUFBBS7;
    uint8_t RESERVED_9[96];
    volatile uint32_t TIMCTL0;
    volatile uint32_t TIMCTL1;
    volatile uint32_t TIMCTL2;
    volatile uint32_t TIMCTL3;
    volatile uint32_t TIMCTL4;
    volatile uint32_t TIMCTL5;
    volatile uint32_t TIMCTL6;
    volatile uint32_t TIMCTL7;
    uint8_t RESERVED_10[96];
    volatile uint32_t TIMCFG0;
    volatile uint32_t TIMCFG1;
    volatile uint32_t TIMCFG2;
    volatile uint32_t TIMCFG3;
    volatile uint32_t TIMCFG4;
    volatile uint32_t TIMCFG5;
    volatile uint32_t TIMCFG6;
    volatile uint32_t TIMCFG7;
    uint8_t RESERVED_11[96];
    volatile uint32_t TIMCMP0;
    volatile uint32_t TIMCMP1;
    volatile uint32_t TIMCMP2;
    volatile uint32_t TIMCMP3;
    volatile uint32_t TIMCMP4;
    volatile uint32_t TIMCMP5;
    volatile uint32_t TIMCMP6;
    volatile uint32_t TIMCMP7;
    uint8_t RESERVED_12[352];
    volatile uint32_t SHIFTBUFNBS0;
    volatile uint32_t SHIFTBUFNBS1;
    volatile uint32_t SHIFTBUFNBS2;
    volatile uint32_t SHIFTBUFNBS3;
    volatile uint32_t SHIFTBUFNBS4;
    volatile uint32_t SHIFTBUFNBS5;
    volatile uint32_t SHIFTBUFNBS6;
    volatile uint32_t SHIFTBUFNBS7;
    uint8_t RESERVED_13[96];
    volatile uint32_t SHIFTBUFHWS0;
    volatile uint32_t SHIFTBUFHWS1;
    volatile uint32_t SHIFTBUFHWS2;
    volatile uint32_t SHIFTBUFHWS3;
    volatile uint32_t SHIFTBUFHWS4;
    volatile uint32_t SHIFTBUFHWS5;
    volatile uint32_t SHIFTBUFHWS6;
    volatile uint32_t SHIFTBUFHWS7;
    uint8_t RESERVED_14[96];
    volatile uint32_t SHIFTBUFNIS0;
    volatile uint32_t SHIFTBUFNIS1;
    volatile uint32_t SHIFTBUFNIS2;
    volatile uint32_t SHIFTBUFNIS3;
    volatile uint32_t SHIFTBUFNIS4;
    volatile uint32_t SHIFTBUFNIS5;
    volatile uint32_t SHIFTBUFNIS6;
    volatile uint32_t SHIFTBUFNIS7;
} FLEXIO_t;

#define FLEXIO1 ((FLEXIO_t *)FLEXIO1_ADDR)
#define FLEXIO2 ((FLEXIO_t *)FLEXIO2_ADDR)
#define FLEXIO3 ((FLEXIO_t *)FLEXIO3_ADDR)

// IOMUX Registers
#define IOMUXC_ADDR 0x401F8014u

typedef struct {
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_00;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_01;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_02;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_03;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_04;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_05;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_06;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_07;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_08;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_09;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_10;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_11;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_12;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_13;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_14;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_15;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_16;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_17;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_18;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_19;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_20;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_21;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_22;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_23;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_24;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_25;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_26;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_27;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_28;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_29;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_30;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_31;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_32;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_33;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_34;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_35;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_36;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_37;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_38;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_39;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_40;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_EMC_41;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_00;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_01;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_02;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_03;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_04;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_05;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_06;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_07;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_08;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_09;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_10;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_11;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_12;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_13;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_14;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B0_15;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_00;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_01;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_02;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_03;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_04;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_05;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_06;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_07;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_08;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_09;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_10;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_11;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_12;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_13;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_14;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_AD_B1_15;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_00;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_01;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_02;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_03;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_04;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_05;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_06;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_07;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_08;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_09;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_10;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_11;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_12;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_13;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_14;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B0_15;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_00;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_01;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_02;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_03;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_04;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_05;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_06;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_07;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_08;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_09;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_10;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_11;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_12;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_13;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_14;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_B1_15;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B0_00;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B0_01;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B0_02;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B0_03;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B0_04;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B0_05;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_00;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_01;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_02;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_03;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_04;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_05;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_06;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_07;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_08;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_09;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_10;
    volatile uint32_t SW_MUX_CTL_PAD_GPIO_SD_B1_11;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_00;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_01;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_02;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_03;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_04;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_05;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_06;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_07;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_08;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_09;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_10;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_11;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_12;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_13;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_14;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_15;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_16;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_17;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_18;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_19;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_20;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_21;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_22;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_23;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_24;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_25;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_26;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_27;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_28;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_29;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_30;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_31;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_32;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_33;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_34;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_35;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_36;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_37;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_38;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_39;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_40;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_EMC_41;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_00;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_01;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_02;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_03;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_04;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_05;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_06;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_07;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_08;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_09;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_10;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_11;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_12;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_13;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_14;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B0_15;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_00;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_01;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_02;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_03;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_04;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_05;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_06;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_07;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_08;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_09;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_10;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_11;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_12;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_13;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_14;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_AD_B1_15;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_00;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_01;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_02;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_03;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_04;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_05;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_06;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_07;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_08;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_09;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_10;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_11;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_12;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_13;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_14;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B0_15;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_00;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_01;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_02;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_03;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_04;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_05;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_06;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_07;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_08;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_09;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_10;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_11;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_12;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_13;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_14;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_B1_15;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B0_00;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B0_01;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B0_02;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B0_03;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B0_04;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B0_05;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_00;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_01;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_02;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_03;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_04;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_05;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_06;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_07;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_08;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_09;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_10;
    volatile uint32_t SW_PAD_CTL_PAD_GPIO_SD_B1_11;
} IOMUXC_t;

#define IOMUXC ((IOMUXC_t *)IOMUXC_ADDR)

// GPIO Registers
#define GPIO1_ADDR 0x401B8000u
#define GPIO2_ADDR 0x401BC000u
#define GPIO3_ADDR 0x401C0000u
#define GPIO4_ADDR 0x401B4000u
#define GPIO5_ADDR 0x400C0000u
#define GPIO6_ADDR 0x42000000u
#define GPIO7_ADDR 0x42004000u
#define GPIO8_ADDR 0x42008000u
#define GPIO9_ADDR 0x4200C000u

typedef struct {
    volatile uint32_t DR;
    volatile uint32_t GDIR;
    volatile uint32_t PSR;
    volatile uint32_t ICR1;
    volatile uint32_t ICR2;
    volatile uint32_t IMR;
    volatile uint32_t ISR;
    volatile uint32_t EDGE_SEL;
    uint8_t RESERVED_0[100];
    volatile uint32_t DR_SET;
    volatile uint32_t DR_CLEAR;
    volatile uint32_t DR_TOGGLE;
} GPIO_t;

#define GPIO1 ((GPIO_t *)GPIO1_ADDR)
#define GPIO2 ((GPIO_t *)GPIO2_ADDR)
#define GPIO3 ((GPIO_t *)GPIO3_ADDR)
#define GPIO4 ((GPIO_t *)GPIO4_ADDR)
#define GPIO5 ((GPIO_t *)GPIO5_ADDR)
#define GPIO6 ((GPIO_t *)GPIO6_ADDR)
#define GPIO7 ((GPIO_t *)GPIO7_ADDR)
#define GPIO8 ((GPIO_t *)GPIO8_ADDR)
#define GPIO9 ((GPIO_t *)GPIO9_ADDR)

// Clock Controller Module Registers
#define CCM_ADDR 0x400FC000u

typedef struct {
    volatile uint32_t CCR;
    uint8_t RESERVED_0[4];
    volatile uint32_t CSR;
    volatile uint32_t CCSR;
    volatile uint32_t CACRR;
    volatile uint32_t CBCDR;
    volatile uint32_t CBCMR;
    volatile uint32_t CSCMR1;
    volatile uint32_t CSCMR2;
    volatile uint32_t CSCDR1;
    volatile uint32_t CS1CDR;
    volatile uint32_t CS2CDR;
    volatile uint32_t CDCDR;
    uint8_t RESERVED_1[4];
    volatile uint32_t CSCDR2;
    volatile uint32_t CSCDR3;
    uint8_t RESERVED_2[8];
    volatile uint32_t CDHIPR;
    uint8_t RESERVED_3[8];
    volatile uint32_t CLPCR;
    volatile uint32_t CISR;
    volatile uint32_t CIMR;
    volatile uint32_t CCOSR;
    volatile uint32_t CGPR;
    volatile uint32_t CCGR0;
    volatile uint32_t CCGR1;
    volatile uint32_t CCGR2;
    volatile uint32_t CCGR3;
    volatile uint32_t CCGR4;
    volatile uint32_t CCGR5;
    volatile uint32_t CCGR6;
    volatile uint32_t CCGR7;
    volatile uint32_t CMEOR;
} CCM_t;

#define CCM ((CCM_t *)CCM_ADDR)

// LPSPI Registers
#define LPSPI1_ADDR 0x40394000u
#define LPSPI2_ADDR 0x40398000u
#define LPSPI3_ADDR 0x4039C000u
#define LPSPI4_ADDR 0x403A0000u

typedef struct {
    volatile uint32_t VERID;
    volatile uint32_t PARAM;
    uint8_t RESERVED_0[8];
    volatile uint32_t CR;
    volatile uint32_t SR;
    volatile uint32_t IER;
    volatile uint32_t DER;
    volatile uint32_t CFGR0;
    volatile uint32_t CFGR1;
    uint8_t RESERVED_1[8];
    volatile uint32_t DMR0;
    volatile uint32_t DMR1;
    uint8_t RESERVED_2[8];
    volatile uint32_t CCR;
    uint8_t RESERVED_3[20];
    volatile uint32_t FCR;
    volatile uint32_t FSR;
    volatile uint32_t TCR;
    volatile uint32_t TDR;
    uint8_t RESERVED_4[8];
    volatile uint32_t RSR;
    volatile uint32_t RDR;
} LPSPI_t;

#define LPSPI1 ((LPSPI_t *)LPSPI1_ADDR)
#define LPSPI2 ((LPSPI_t *)LPSPI2_ADDR)
#define LPSPI3 ((LPSPI_t *)LPSPI3_ADDR)
#define LPSPI4 ((LPSPI_t *)LPSPI4_ADDR)

// LPI2C Registers

#define LPI2C1_ADDR 0x403F0000u
#define LPI2C2_ADDR 0x403F4000u
#define LPI2C3_ADDR 0x403F8000u
#define LPI2C4_ADDR 0x403FC000u

typedef struct {
    volatile uint32_t VERID;
    volatile uint32_t PARAM;
    uint8_t RESERVED_0[8];
    volatile uint32_t MCR;
    volatile uint32_t MSR;
    volatile uint32_t MIER;
    volatile uint32_t MDER;
    volatile uint32_t MCFGR0;
    volatile uint32_t MCFGR1;
    volatile uint32_t MCFGR2;
    volatile uint32_t MCFGR3;
    uint8_t RESERVED_1[16];
    volatile uint32_t MDMR;
    uint8_t RESERVED_2[4];
    volatile uint32_t MCCR0;
    uint8_t RESERVED_3[4];
    volatile uint32_t MCCR1;
    uint8_t RESERVED_4[4];
    volatile uint32_t MFCR;
    volatile uint32_t MFSR;
    volatile uint32_t MTDR;
    uint8_t RESERVED_5[12];
    volatile uint32_t MRDR;
    uint8_t RESERVED_6[156];
    volatile uint32_t SCR;
    volatile uint32_t SSR;
    volatile uint32_t SIER;
    volatile uint32_t SDER;
    uint8_t RESERVED_7[4];
    volatile uint32_t SCFGR1;
    volatile uint32_t SCFGR2;
    uint8_t RESERVED_8[20];
    volatile uint32_t SAMR;
    uint8_t RESERVED_9[12];
    volatile uint32_t SASR;
    volatile uint32_t STAR;
    uint8_t RESERVED_10[8];
    volatile uint32_t STDR;
    uint8_t RESERVED_11[12];
    volatile uint32_t SRDR;
} LPI2C_t;

#define LPI2C1 ((LPI2C_t *)LPI2C1_ADDR)
#define LPI2C2 ((LPI2C_t *)LPI2C2_ADDR)
#define LPI2C3 ((LPI2C_t *)LPI2C3_ADDR)
#define LPI2C4 ((LPI2C_t *)LPI2C4_ADDR)

// LPUART Registers
#define LPUART1_ADDR 0x40184000u
#define LPUART2_ADDR 0x40188000u
#define LPUART3_ADDR 0x4018C000u
#define LPUART4_ADDR 0x40190000u
#define LPUART5_ADDR 0x40194000u
#define LPUART6_ADDR 0x40198000u
#define LPUART7_ADDR 0x4019C000u
#define LPUART8_ADDR 0x401A0000u

typedef struct {
    volatile uint32_t VERID;
    volatile uint32_t PARAM;
    volatile uint32_t GLOBAL;
    volatile uint32_t PINCFG;
    volatile uint32_t BAUD;
    volatile uint32_t STAT;
    volatile uint32_t CTRL;
    volatile uint32_t DATA;
    volatile uint32_t MATCH;
    volatile uint32_t MODIR;
    volatile uint32_t FIFO;
    volatile uint32_t WATER;
} LPUART_t;

#define LPUART1 ((LPUART_t *)LPUART1_ADDR)
#define LPUART2 ((LPUART_t *)LPUART2_ADDR)
#define LPUART3 ((LPUART_t *)LPUART3_ADDR)
#define LPUART4 ((LPUART_t *)LPUART4_ADDR)
#define LPUART5 ((LPUART_t *)LPUART5_ADDR)
#define LPUART6 ((LPUART_t *)LPUART6_ADDR)
#define LPUART7 ((LPUART_t *)LPUART7_ADDR)
#define LPUART8 ((LPUART_t *)LPUART8_ADDR)

// DMAMUX Registers

#define DMAMUX_ADDR 0x400EC000u

#define DMAMUX_CHCFG(n) *((volatile uint32_t *)DMAMUX_ADDR + 4 * (n))

// DMA Registers

#define DMA_ADDR 0x400E8000u

#define DMA_CR *((volatile uint32_t *)DMA_ADDR + 0x4)
#define DMA_ERQ *((volatile uint32_t *)DMA_ADDR + 0xC)
#define DMA_EEI *((volatile uint32_t *)DMA_ADDR + 0x14)
#define DMA_CEEI *((volatile uint8_t *)DMA_ADDR + 0x18)
#define DMA_SEEI *((volatile uint8_t *)DMA_ADDR + 0x19)
#define DMA_CERQ *((volatile uint8_t *)DMA_ADDR + 0x1A)
#define DMA_SERQ *((volatile uint8_t *)DMA_ADDR + 0x1B)
#define DMA_CDNE *((volatile uint8_t *)DMA_ADDR + 0x1C)
#define DMA_SSRT *((volatile uint8_t *)DMA_ADDR + 0x1D)
#define DMA_CERR *((volatile uint8_t *)DMA_ADDR + 0x1E)
#define DMA_CINT *((volatile uint8_t *)DMA_ADDR + 0x1F)
#define DMA_INT *((volatile uint32_t *)DMA_ADDR + 0x24)
#define DMA_ERR *((volatile uint32_t *)DMA_ADDR + 0x2C)
#define DMA_HRS *((volatile uint32_t *)DMA_ADDR + 0x34)
#define DMA_EARS *((volatile uint32_t *)DMA_ADDR + 0x44)
#define DMA_DCHPRI3 *((volatile uint8_t *)DMA_ADDR + 0x100)
#define DMA_DCHPRI2 *((volatile uint8_t *)DMA_ADDR + 0x101)
#define DMA_DCHPRI1 *((volatile uint8_t *)DMA_ADDR + 0x102)
#define DMA_DCHPRI0 *((volatile uint8_t *)DMA_ADDR + 0x103)
#define DMA_DCHPRI7 *((volatile uint8_t *)DMA_ADDR + 0x104)
#define DMA_DCHPRI6 *((volatile uint8_t *)DMA_ADDR + 0x105)
#define DMA_DCHPRI5 *((volatile uint8_t *)DMA_ADDR + 0x106)
#define DMA_DCHPRI4 *((volatile uint8_t *)DMA_ADDR + 0x107)
#define DMA_DCHPRI11 *((volatile uint8_t *)DMA_ADDR + 0x108)
#define DMA_DCHPRI10 *((volatile uint8_t *)DMA_ADDR + 0x109)
#define DMA_DCHPRI9 *((volatile uint8_t *)DMA_ADDR + 0x10A)
#define DMA_DCHPRI8 *((volatile uint8_t *)DMA_ADDR + 0x10B)
#define DMA_DCHPRI15 *((volatile uint8_t *)DMA_ADDR + 0x10C)
#define DMA_DCHPRI14 *((volatile uint8_t *)DMA_ADDR + 0x10D)
#define DMA_DCHPRI13 *((volatile uint8_t *)DMA_ADDR + 0x10E)
#define DMA_DCHPRI12 *((volatile uint8_t *)DMA_ADDR + 0x10F)
#define DMA_DCHPRI19 *((volatile uint8_t *)DMA_ADDR + 0x110)
#define DMA_DCHPRI18 *((volatile uint8_t *)DMA_ADDR + 0x111)
#define DMA_DCHPRI17 *((volatile uint8_t *)DMA_ADDR + 0x112)
#define DMA_DCHPRI16 *((volatile uint8_t *)DMA_ADDR + 0x113)
#define DMA_DCHPRI23 *((volatile uint8_t *)DMA_ADDR + 0x114)
#define DMA_DCHPRI22 *((volatile uint8_t *)DMA_ADDR + 0x115)
#define DMA_DCHPRI21 *((volatile uint8_t *)DMA_ADDR + 0x116)
#define DMA_DCHPRI20 *((volatile uint8_t *)DMA_ADDR + 0x117)
#define DMA_DCHPRI27 *((volatile uint8_t *)DMA_ADDR + 0x118)
#define DMA_DCHPRI26 *((volatile uint8_t *)DMA_ADDR + 0x119)
#define DMA_DCHPRI25 *((volatile uint8_t *)DMA_ADDR + 0x11A)
#define DMA_DCHPRI24 *((volatile uint8_t *)DMA_ADDR + 0x11B)
#define DMA_DCHPRI31 *((volatile uint8_t *)DMA_ADDR + 0x11C)
#define DMA_DCHPRI30 *((volatile uint8_t *)DMA_ADDR + 0x11D)
#define DMA_DCHPRI29 *((volatile uint8_t *)DMA_ADDR + 0x11E)
#define DMA_DCHPRI28 *((volatile uint8_t *)DMA_ADDR + 0x11F)
#define DMA_TCD_SADDR(n) *((volatile uint32_t *)DMA_ADDR + 0x1000 + (n)*0x20)
#define DMA_TCD_SOFF(n) *((volatile uint16_t *)DMA_ADDR + 0x1004 + (n)*0x20)
#define DMA_TCD_ATTR(n) *((volatile uint16_t *)DMA_ADDR + 0x1006 + (n)*0x20)
#define DMA_TCD_NBYTES(n) *((volatile uint32_t *)DMA_ADDR + 0x1008 + (n)*0x20)
#define DMA_TCD_SLAST(n) *((volatile uint32_t *)DMA_ADDR + 0x100C + (n)*0x20)
#define DMA_TCD_DADDR(n) *((volatile uint32_t *)DMA_ADDR + 0x1010 + (n)*0x20)
#define DMA_TCD_DOFF(n) *((volatile uint16_t *)DMA_ADDR + 0x1014 + (n)*0x20)
#define DMA_TCD_CITER(n) *((volatile uint16_t *)DMA_ADDR + 0x1016 + (n)*0x20)
#define DMA_TCD_DLASTSGA(n) *((volatile uint32_t *)DMA_ADDR + 0x1018 + (n)*0x20)
#define DMA_TCD_CSR(n) *((volatile uint16_t *)DMA_ADDR + 0x101C + (n)*0x20)
#define DMA_TCD_BITER(n) *((volatile uint16_t *)DMA_ADDR + 0x101E + (n)*0x20)

// System Control Block

#define SCB_ADDR 0xE000ED00u

typedef struct {
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint32_t SHPR1;
    volatile uint32_t SHPR2;
    volatile uint32_t SHPR3;
    volatile uint32_t SHCRS;
    volatile uint32_t CFSR;
    volatile uint32_t HFSR;
    uint8_t RESERVED_0[4];
    volatile uint32_t MMAR;
    volatile uint32_t BFAR;
    volatile uint32_t AFSR;
} SCB_t;

#define SCB ((SCB_t *)SCB_ADDR)

// Pin definitions

typedef struct {
    volatile uint32_t *MUX_REG_ADDR;
    volatile uint32_t *PAD_REG_ADDR;
    int16_t flexio_pin;
    int16_t gpio_pin;
} pinInfo_t;

#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY40)

#define PIN0_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_03
#define PIN1_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_02
#define PIN2_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_04
#define PIN3_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_05
#define PIN4_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_06
#define PIN5_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_08
#define PIN6_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_10
#define PIN7_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_01
#define PIN8_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_00
#define PIN9_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_11
#define PIN10_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_00
#define PIN11_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_02
#define PIN12_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_01
#define PIN13_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_03
#define PIN14_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_02
#define PIN15_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_03
#define PIN16_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_07
#define PIN17_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_06
#define PIN18_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_01
#define PIN19_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_00
#define PIN20_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_10
#define PIN21_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_11
#define PIN22_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_08
#define PIN23_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_09
#define PIN24_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_12
#define PIN25_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_13
#define PIN26_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_14
#define PIN27_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_15
#define PIN28_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_32
#define PIN29_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_31
#define PIN30_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_37
#define PIN31_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_36
#define PIN32_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_12
#define PIN33_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_07
#define PIN34_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_03
#define PIN35_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_02
#define PIN36_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_01
#define PIN37_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_00
#define PIN38_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_05
#define PIN39_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_04
#define PIN0_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_03
#define PIN1_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_02
#define PIN2_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_04
#define PIN3_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_05
#define PIN4_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_06
#define PIN5_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_08
#define PIN6_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_10
#define PIN7_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_01
#define PIN8_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_00
#define PIN9_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_11
#define PIN10_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_00
#define PIN11_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_02
#define PIN12_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_01
#define PIN13_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_03
#define PIN14_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_02
#define PIN15_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_03
#define PIN16_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_07
#define PIN17_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_06
#define PIN18_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_01
#define PIN19_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_00
#define PIN20_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_10
#define PIN21_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_11
#define PIN22_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_08
#define PIN23_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_09
#define PIN24_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_12
#define PIN25_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_13
#define PIN26_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_14
#define PIN27_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_15
#define PIN28_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_32
#define PIN29_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_31
#define PIN30_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_37
#define PIN31_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_36
#define PIN32_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_12
#define PIN33_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_07
#define PIN34_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_03
#define PIN35_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_02
#define PIN36_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_01
#define PIN37_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_00
#define PIN38_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_05
#define PIN39_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_04

static const pinInfo_t PIN[] = {
    {.MUX_REG_ADDR = &PIN0_MUX,
     .PAD_REG_ADDR = &PIN0_PAD,
     .flexio_pin = -1,
     .gpio_pin = 103},
    {.MUX_REG_ADDR = &PIN1_MUX,
     .PAD_REG_ADDR = &PIN1_PAD,
     .flexio_pin = -1,
     .gpio_pin = 102},
    {.MUX_REG_ADDR = &PIN2_MUX,
     .PAD_REG_ADDR = &PIN2_PAD,
     .flexio_pin = 104,
     .gpio_pin = 404},
    {.MUX_REG_ADDR = &PIN3_MUX,
     .PAD_REG_ADDR = &PIN3_PAD,
     .flexio_pin = 105,
     .gpio_pin = 405},
    {.MUX_REG_ADDR = &PIN4_MUX,
     .PAD_REG_ADDR = &PIN4_PAD,
     .flexio_pin = 106,
     .gpio_pin = 406},
    {.MUX_REG_ADDR = &PIN5_MUX,
     .PAD_REG_ADDR = &PIN5_PAD,
     .flexio_pin = 108,
     .gpio_pin = 408},
    {.MUX_REG_ADDR = &PIN6_MUX,
     .PAD_REG_ADDR = &PIN6_PAD,
     .flexio_pin = 210,
     .gpio_pin = 210},
    {.MUX_REG_ADDR = &PIN7_MUX,
     .PAD_REG_ADDR = &PIN7_PAD,
     .flexio_pin = 217,
     .gpio_pin = 217},
    {.MUX_REG_ADDR = &PIN8_MUX,
     .PAD_REG_ADDR = &PIN8_PAD,
     .flexio_pin = 216,
     .gpio_pin = 216},
    {.MUX_REG_ADDR = &PIN9_MUX,
     .PAD_REG_ADDR = &PIN9_PAD,
     .flexio_pin = 211,
     .gpio_pin = 211},
    {.MUX_REG_ADDR = &PIN10_MUX,
     .PAD_REG_ADDR = &PIN10_PAD,
     .flexio_pin = 200,
     .gpio_pin = 200},
    {.MUX_REG_ADDR = &PIN11_MUX,
     .PAD_REG_ADDR = &PIN11_PAD,
     .flexio_pin = 202,
     .gpio_pin = 202},
    {.MUX_REG_ADDR = &PIN12_MUX,
     .PAD_REG_ADDR = &PIN12_PAD,
     .flexio_pin = 201,
     .gpio_pin = 201},
    {.MUX_REG_ADDR = &PIN13_MUX,
     .PAD_REG_ADDR = &PIN13_PAD,
     .flexio_pin = 203,
     .gpio_pin = 203},
    {.MUX_REG_ADDR = &PIN14_MUX,
     .PAD_REG_ADDR = &PIN14_PAD,
     .flexio_pin = 302,
     .gpio_pin = 118},
    {.MUX_REG_ADDR = &PIN15_MUX,
     .PAD_REG_ADDR = &PIN15_PAD,
     .flexio_pin = 303,
     .gpio_pin = 119},
    {.MUX_REG_ADDR = &PIN16_MUX,
     .PAD_REG_ADDR = &PIN16_PAD,
     .flexio_pin = 307,
     .gpio_pin = 123},
    {.MUX_REG_ADDR = &PIN17_MUX,
     .PAD_REG_ADDR = &PIN17_PAD,
     .flexio_pin = 306,
     .gpio_pin = 122},
    {.MUX_REG_ADDR = &PIN18_MUX,
     .PAD_REG_ADDR = &PIN18_PAD,
     .flexio_pin = 301,
     .gpio_pin = 117},
    {.MUX_REG_ADDR = &PIN19_MUX,
     .PAD_REG_ADDR = &PIN19_PAD,
     .flexio_pin = 300,
     .gpio_pin = 116},
    {.MUX_REG_ADDR = &PIN20_MUX,
     .PAD_REG_ADDR = &PIN20_PAD,
     .flexio_pin = 310,
     .gpio_pin = 126},
    {.MUX_REG_ADDR = &PIN21_MUX,
     .PAD_REG_ADDR = &PIN21_PAD,
     .flexio_pin = 311,
     .gpio_pin = 127},
    {.MUX_REG_ADDR = &PIN22_MUX,
     .PAD_REG_ADDR = &PIN22_PAD,
     .flexio_pin = 308,
     .gpio_pin = 124},
    {.MUX_REG_ADDR = &PIN23_MUX,
     .PAD_REG_ADDR = &PIN23_PAD,
     .flexio_pin = 309,
     .gpio_pin = 125},
    {.MUX_REG_ADDR = &PIN24_MUX,
     .PAD_REG_ADDR = &PIN24_PAD,
     .flexio_pin = -1,
     .gpio_pin = 112},
    {.MUX_REG_ADDR = &PIN25_MUX,
     .PAD_REG_ADDR = &PIN25_PAD,
     .flexio_pin = -1,
     .gpio_pin = 113},
    {.MUX_REG_ADDR = &PIN26_MUX,
     .PAD_REG_ADDR = &PIN26_PAD,
     .flexio_pin = 314,
     .gpio_pin = 130},
    {.MUX_REG_ADDR = &PIN27_MUX,
     .PAD_REG_ADDR = &PIN27_PAD,
     .flexio_pin = 315,
     .gpio_pin = 131},
    {.MUX_REG_ADDR = &PIN28_MUX,
     .PAD_REG_ADDR = &PIN28_PAD,
     .flexio_pin = -1,
     .gpio_pin = 318},
    {.MUX_REG_ADDR = &PIN29_MUX,
     .PAD_REG_ADDR = &PIN29_PAD,
     .flexio_pin = -1,
     .gpio_pin = 431},
    {.MUX_REG_ADDR = &PIN30_MUX,
     .PAD_REG_ADDR = &PIN30_PAD,
     .flexio_pin = -1,
     .gpio_pin = 323},
    {.MUX_REG_ADDR = &PIN31_MUX,
     .PAD_REG_ADDR = &PIN31_PAD,
     .flexio_pin = -1,
     .gpio_pin = 322},
    {.MUX_REG_ADDR = &PIN32_MUX,
     .PAD_REG_ADDR = &PIN32_PAD,
     .flexio_pin = 212,
     .gpio_pin = 212},
    {.MUX_REG_ADDR = &PIN33_MUX,
     .PAD_REG_ADDR = &PIN33_PAD,
     .flexio_pin = 107,
     .gpio_pin = 407},
    {.MUX_REG_ADDR = &PIN34_MUX,
     .PAD_REG_ADDR = &PIN34_PAD,
     .flexio_pin = -1,
     .gpio_pin = 315},
    {.MUX_REG_ADDR = &PIN35_MUX,
     .PAD_REG_ADDR = &PIN35_PAD,
     .flexio_pin = -1,
     .gpio_pin = 314},
    {.MUX_REG_ADDR = &PIN36_MUX,
     .PAD_REG_ADDR = &PIN36_PAD,
     .flexio_pin = -1,
     .gpio_pin = 313},
    {.MUX_REG_ADDR = &PIN37_MUX,
     .PAD_REG_ADDR = &PIN37_PAD,
     .flexio_pin = -1,
     .gpio_pin = 312},
    {.MUX_REG_ADDR = &PIN38_MUX,
     .PAD_REG_ADDR = &PIN38_PAD,
     .flexio_pin = -1,
     .gpio_pin = 317},
    {.MUX_REG_ADDR = &PIN39_MUX,
     .PAD_REG_ADDR = &PIN39_PAD,
     .flexio_pin = -1,
     .gpio_pin = 316},
};

#elif defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41)

#define PIN0_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_03
#define PIN1_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_02
#define PIN2_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_04
#define PIN3_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_05
#define PIN4_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_06
#define PIN5_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_08
#define PIN6_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_10
#define PIN7_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_01
#define PIN8_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_00
#define PIN9_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_11
#define PIN10_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_00
#define PIN11_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_02
#define PIN12_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_01
#define PIN13_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_03
#define PIN14_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_02
#define PIN15_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_03
#define PIN16_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_07
#define PIN17_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_06
#define PIN18_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_01
#define PIN19_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_00
#define PIN20_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_10
#define PIN21_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_11
#define PIN22_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_08
#define PIN23_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_09
#define PIN24_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_12
#define PIN25_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_13
#define PIN26_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_14
#define PIN27_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_15
#define PIN28_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_32
#define PIN29_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_31
#define PIN30_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_37
#define PIN31_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_36
#define PIN32_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_12
#define PIN33_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_07
#define PIN34_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_13
#define PIN35_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_12
#define PIN36_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_02
#define PIN37_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_03
#define PIN38_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_12
#define PIN39_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_13
#define PIN40_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_04
#define PIN41_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_05
#define PIN42_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_03
#define PIN43_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_02
#define PIN44_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_01
#define PIN45_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_00
#define PIN46_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_05
#define PIN47_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_04
#define PIN48_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_24
#define PIN49_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_27
#define PIN50_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_28
#define PIN51_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_22
#define PIN52_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_26
#define PIN53_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_25
#define PIN54_MUX IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_29

#define PIN0_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_03
#define PIN1_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_02
#define PIN2_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_04
#define PIN3_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_05
#define PIN4_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_06
#define PIN5_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_08
#define PIN6_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_10
#define PIN7_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_01
#define PIN8_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_00
#define PIN9_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_11
#define PIN10_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_00
#define PIN11_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_02
#define PIN12_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_01
#define PIN13_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_03
#define PIN14_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_02
#define PIN15_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_03
#define PIN16_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_07
#define PIN17_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_06
#define PIN18_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_01
#define PIN19_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_00
#define PIN20_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_10
#define PIN21_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_11
#define PIN22_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_08
#define PIN23_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_09
#define PIN24_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_12
#define PIN25_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_13
#define PIN26_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_14
#define PIN27_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_15
#define PIN28_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_32
#define PIN29_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_31
#define PIN30_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_37
#define PIN31_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_36
#define PIN32_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_12
#define PIN33_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_07
#define PIN34_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_13
#define PIN35_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_12
#define PIN36_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_02
#define PIN37_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_03
#define PIN38_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_12
#define PIN39_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_13
#define PIN40_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_04
#define PIN41_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_05
#define PIN42_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_03
#define PIN43_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_02
#define PIN44_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_01
#define PIN45_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_00
#define PIN46_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_05
#define PIN47_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_04
#define PIN48_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_24
#define PIN49_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_27
#define PIN50_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_28
#define PIN51_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_22
#define PIN52_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_26
#define PIN53_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_25
#define PIN54_PAD IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_29

static const pinInfo_t PIN[] = {
    {.MUX_REG_ADDR = &PIN0_MUX,
     .PAD_REG_ADDR = &PIN0_PAD,
     .flexio_pin = -1,
     .gpio_pin = 103},
    {.MUX_REG_ADDR = &PIN1_MUX,
     .PAD_REG_ADDR = &PIN1_PAD,
     .flexio_pin = -1,
     .gpio_pin = 102},
    {.MUX_REG_ADDR = &PIN2_MUX,
     .PAD_REG_ADDR = &PIN2_PAD,
     .flexio_pin = 104,
     .gpio_pin = 404},
    {.MUX_REG_ADDR = &PIN3_MUX,
     .PAD_REG_ADDR = &PIN3_PAD,
     .flexio_pin = 105,
     .gpio_pin = 405},
    {.MUX_REG_ADDR = &PIN4_MUX,
     .PAD_REG_ADDR = &PIN4_PAD,
     .flexio_pin = 106,
     .gpio_pin = 406},
    {.MUX_REG_ADDR = &PIN5_MUX,
     .PAD_REG_ADDR = &PIN5_PAD,
     .flexio_pin = 108,
     .gpio_pin = 408},
    {.MUX_REG_ADDR = &PIN6_MUX,
     .PAD_REG_ADDR = &PIN6_PAD,
     .flexio_pin = 210,
     .gpio_pin = 210},
    {.MUX_REG_ADDR = &PIN7_MUX,
     .PAD_REG_ADDR = &PIN7_PAD,
     .flexio_pin = 217,
     .gpio_pin = 217},
    {.MUX_REG_ADDR = &PIN8_MUX,
     .PAD_REG_ADDR = &PIN8_PAD,
     .flexio_pin = 216,
     .gpio_pin = 216},
    {.MUX_REG_ADDR = &PIN9_MUX,
     .PAD_REG_ADDR = &PIN9_PAD,
     .flexio_pin = 211,
     .gpio_pin = 211},
    {.MUX_REG_ADDR = &PIN10_MUX,
     .PAD_REG_ADDR = &PIN10_PAD,
     .flexio_pin = 200,
     .gpio_pin = 200},
    {.MUX_REG_ADDR = &PIN11_MUX,
     .PAD_REG_ADDR = &PIN11_PAD,
     .flexio_pin = 202,
     .gpio_pin = 202},
    {.MUX_REG_ADDR = &PIN12_MUX,
     .PAD_REG_ADDR = &PIN12_PAD,
     .flexio_pin = 201,
     .gpio_pin = 201},
    {.MUX_REG_ADDR = &PIN13_MUX,
     .PAD_REG_ADDR = &PIN13_PAD,
     .flexio_pin = 203,
     .gpio_pin = 203},
    {.MUX_REG_ADDR = &PIN14_MUX,
     .PAD_REG_ADDR = &PIN14_PAD,
     .flexio_pin = 301,
     .gpio_pin = 118},
    {.MUX_REG_ADDR = &PIN15_MUX,
     .PAD_REG_ADDR = &PIN15_PAD,
     .flexio_pin = 303,
     .gpio_pin = 119},
    {.MUX_REG_ADDR = &PIN16_MUX,
     .PAD_REG_ADDR = &PIN16_PAD,
     .flexio_pin = 307,
     .gpio_pin = 123},
    {.MUX_REG_ADDR = &PIN17_MUX,
     .PAD_REG_ADDR = &PIN17_PAD,
     .flexio_pin = 306,
     .gpio_pin = 122},
    {.MUX_REG_ADDR = &PIN18_MUX,
     .PAD_REG_ADDR = &PIN18_PAD,
     .flexio_pin = 301,
     .gpio_pin = 117},
    {.MUX_REG_ADDR = &PIN19_MUX,
     .PAD_REG_ADDR = &PIN19_PAD,
     .flexio_pin = 300,
     .gpio_pin = 116},
    {.MUX_REG_ADDR = &PIN20_MUX,
     .PAD_REG_ADDR = &PIN20_PAD,
     .flexio_pin = 310,
     .gpio_pin = 126},
    {.MUX_REG_ADDR = &PIN21_MUX,
     .PAD_REG_ADDR = &PIN21_PAD,
     .flexio_pin = 311,
     .gpio_pin = 127},
    {.MUX_REG_ADDR = &PIN22_MUX,
     .PAD_REG_ADDR = &PIN22_PAD,
     .flexio_pin = 308,
     .gpio_pin = 124},
    {.MUX_REG_ADDR = &PIN23_MUX,
     .PAD_REG_ADDR = &PIN23_PAD,
     .flexio_pin = 309,
     .gpio_pin = 125},
    {.MUX_REG_ADDR = &PIN24_MUX,
     .PAD_REG_ADDR = &PIN24_PAD,
     .flexio_pin = -1,
     .gpio_pin = 112},
    {.MUX_REG_ADDR = &PIN25_MUX,
     .PAD_REG_ADDR = &PIN25_PAD,
     .flexio_pin = -1,
     .gpio_pin = 113},
    {.MUX_REG_ADDR = &PIN26_MUX,
     .PAD_REG_ADDR = &PIN26_PAD,
     .flexio_pin = 314,
     .gpio_pin = 130},
    {.MUX_REG_ADDR = &PIN27_MUX,
     .PAD_REG_ADDR = &PIN27_PAD,
     .flexio_pin = 315,
     .gpio_pin = 131},
    {.MUX_REG_ADDR = &PIN28_MUX,
     .PAD_REG_ADDR = &PIN28_PAD,
     .flexio_pin = -1,
     .gpio_pin = 318},
    {.MUX_REG_ADDR = &PIN29_MUX,
     .PAD_REG_ADDR = &PIN29_PAD,
     .flexio_pin = -1,
     .gpio_pin = 431},
    {.MUX_REG_ADDR = &PIN30_MUX,
     .PAD_REG_ADDR = &PIN30_PAD,
     .flexio_pin = -1,
     .gpio_pin = 323},
    {.MUX_REG_ADDR = &PIN31_MUX,
     .PAD_REG_ADDR = &PIN31_PAD,
     .flexio_pin = -1,
     .gpio_pin = 322},
    {.MUX_REG_ADDR = &PIN32_MUX,
     .PAD_REG_ADDR = &PIN32_PAD,
     .flexio_pin = 212,
     .gpio_pin = 212},
    {.MUX_REG_ADDR = &PIN33_MUX,
     .PAD_REG_ADDR = &PIN33_PAD,
     .flexio_pin = 107,
     .gpio_pin = 407},
    {.MUX_REG_ADDR = &PIN34_MUX,
     .PAD_REG_ADDR = &PIN34_PAD,
     .flexio_pin = 229,
     .gpio_pin = 229},
    {.MUX_REG_ADDR = &PIN35_MUX,
     .PAD_REG_ADDR = &PIN35_PAD,
     .flexio_pin = 228,
     .gpio_pin = 228},
    {.MUX_REG_ADDR = &PIN36_MUX,
     .PAD_REG_ADDR = &PIN36_PAD,
     .flexio_pin = 218,
     .gpio_pin = 218},
    {.MUX_REG_ADDR = &PIN37_MUX,
     .PAD_REG_ADDR = &PIN37_PAD,
     .flexio_pin = 219,
     .gpio_pin = 219},
    {.MUX_REG_ADDR = &PIN38_MUX,
     .PAD_REG_ADDR = &PIN38_PAD,
     .flexio_pin = 312,
     .gpio_pin = 128},
    {.MUX_REG_ADDR = &PIN39_MUX,
     .PAD_REG_ADDR = &PIN39_PAD,
     .flexio_pin = 313,
     .gpio_pin = 129},
    {.MUX_REG_ADDR = &PIN40_MUX,
     .PAD_REG_ADDR = &PIN40_PAD,
     .flexio_pin = 304,
     .gpio_pin = 120},
    {.MUX_REG_ADDR = &PIN41_MUX,
     .PAD_REG_ADDR = &PIN41_PAD,
     .flexio_pin = 305,
     .gpio_pin = 121},
    {.MUX_REG_ADDR = &PIN42_MUX,
     .PAD_REG_ADDR = &PIN42_PAD,
     .flexio_pin = -1,
     .gpio_pin = 315},
    {.MUX_REG_ADDR = &PIN43_MUX,
     .PAD_REG_ADDR = &PIN43_PAD,
     .flexio_pin = -1,
     .gpio_pin = 314},
    {.MUX_REG_ADDR = &PIN44_MUX,
     .PAD_REG_ADDR = &PIN44_PAD,
     .flexio_pin = -1,
     .gpio_pin = 313},
    {.MUX_REG_ADDR = &PIN45_MUX,
     .PAD_REG_ADDR = &PIN45_PAD,
     .flexio_pin = -1,
     .gpio_pin = 312},
    {.MUX_REG_ADDR = &PIN46_MUX,
     .PAD_REG_ADDR = &PIN46_PAD,
     .flexio_pin = -1,
     .gpio_pin = 317},
    {.MUX_REG_ADDR = &PIN47_MUX,
     .PAD_REG_ADDR = &PIN47_PAD,
     .flexio_pin = -1,
     .gpio_pin = 316},
    {.MUX_REG_ADDR = &PIN48_MUX,
     .PAD_REG_ADDR = &PIN48_PAD,
     .flexio_pin = -1,
     .gpio_pin = 424},
    {.MUX_REG_ADDR = &PIN49_MUX,
     .PAD_REG_ADDR = &PIN49_PAD,
     .flexio_pin = 113,
     .gpio_pin = 427},
    {.MUX_REG_ADDR = &PIN50_MUX,
     .PAD_REG_ADDR = &PIN50_PAD,
     .flexio_pin = 114,
     .gpio_pin = 428},
    {.MUX_REG_ADDR = &PIN51_MUX,
     .PAD_REG_ADDR = &PIN51_PAD,
     .flexio_pin = -1,
     .gpio_pin = 422},
    {.MUX_REG_ADDR = &PIN52_MUX,
     .PAD_REG_ADDR = &PIN52_PAD,
     .flexio_pin = 112,
     .gpio_pin = 426},
    {.MUX_REG_ADDR = &PIN53_MUX,
     .PAD_REG_ADDR = &PIN53_PAD,
     .flexio_pin = -1,
     .gpio_pin = 425},
    {.MUX_REG_ADDR = &PIN54_MUX,
     .PAD_REG_ADDR = &PIN54_PAD,
     .flexio_pin = 115,
     .gpio_pin = 429},
};

#endif