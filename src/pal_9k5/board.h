/*
    Board specific definitions such as pin assignments
*/

#ifndef BOARD_H
#define BOARD_H

#include "stm32h7xx_hal.h"

// Port to index
#define PIN_PA0 0
#define PIN_PA1 1
#define PIN_PA2 2
#define PIN_PA3 3
#define PIN_PA4 4
#define PIN_PA5 5
#define PIN_PA6 6
#define PIN_PA7 7
#define PIN_PA8 8
#define PIN_PA9 9
#define PIN_PA10 10
#define PIN_PA11 11
#define PIN_PA12 12
#define PIN_PA13 13
#define PIN_PA14 14
#define PIN_PA15 15
#define PIN_PB0 16
#define PIN_PB1 17
#define PIN_PB2 18
#define PIN_PB3 19
#define PIN_PB4 20
#define PIN_PB5 21
#define PIN_PB6 22
#define PIN_PB7 23
#define PIN_PB8 24
#define PIN_PB9 25
#define PIN_PB10 26
#define PIN_PB11 27
#define PIN_PB12 28
#define PIN_PB13 29
#define PIN_PB14 30
#define PIN_PB15 31
#define PIN_PC0 32
#define PIN_PC1 33
#define PIN_PC2 34
#define PIN_PC3 35
#define PIN_PC4 36
#define PIN_PC5 37
#define PIN_PC6 38
#define PIN_PC7 39
#define PIN_PC8 40
#define PIN_PC9 41
#define PIN_PC10 42
#define PIN_PC11 43
#define PIN_PC12 44
#define PIN_PC13 45
#define PIN_PC14 46
#define PIN_PC15 47
#define PIN_PD0 48
#define PIN_PD1 49
#define PIN_PD2 50
#define PIN_PD3 51
#define PIN_PD4 52
#define PIN_PD5 53
#define PIN_PD6 54
#define PIN_PD7 55
#define PIN_PD8 56
#define PIN_PD9 57
#define PIN_PD10 58
#define PIN_PD11 59
#define PIN_PD12 60
#define PIN_PD13 61
#define PIN_PD14 62
#define PIN_PD15 63
#define PIN_PE0 64
#define PIN_PE1 65
#define PIN_PE2 66
#define PIN_PE3 67
#define PIN_PE4 68
#define PIN_PE5 69
#define PIN_PE6 70
#define PIN_PE7 71
#define PIN_PE8 72
#define PIN_PE9 73
#define PIN_PE10 74
#define PIN_PE11 75
#define PIN_PE12 76
#define PIN_PE13 77
#define PIN_PE14 78
#define PIN_PE15 79
#define PIN_PF0 80
#define PIN_PF1 81
#define PIN_PF2 82
#define PIN_PF3 83
#define PIN_PF4 84
#define PIN_PF5 85
#define PIN_PF6 86
#define PIN_PF7 87
#define PIN_PF8 88
#define PIN_PF9 89
#define PIN_PF10 90
#define PIN_PF11 91
#define PIN_PF12 92
#define PIN_PF13 93
#define PIN_PF14 94
#define PIN_PF15 95
#define PIN_PG0 96
#define PIN_PG1 97
#define PIN_PG2 98
#define PIN_PG3 99
#define PIN_PG4 100
#define PIN_PG5 101
#define PIN_PG6 102
#define PIN_PG7 103
#define PIN_PG8 104
#define PIN_PG9 105
#define PIN_PG10 106
#define PIN_PG11 107
#define PIN_PG12 108
#define PIN_PG13 109
#define PIN_PG14 110
#define PIN_PG15 111
#define PIN_PH0 112
#define PIN_PH1 113
#define PIN_PH2 114
#define PIN_PH3 115
#define PIN_PH4 116
#define PIN_PH5 117
#define PIN_PH6 118
#define PIN_PH7 119
#define PIN_PH8 120
#define PIN_PH9 121
#define PIN_PH10 122
#define PIN_PH11 123
#define PIN_PH12 124
#define PIN_PH13 125
#define PIN_PH14 126
#define PIN_PH15 127
#define PIN_PI0 128
#define PIN_PI1 129
#define PIN_PI2 130
#define PIN_PI3 131
#define PIN_PI4 132
#define PIN_PI5 133
#define PIN_PI6 134
#define PIN_PI7 135
#define PIN_PI8 136
#define PIN_PI9 137
#define PIN_PI10 138
#define PIN_PI11 139
#define PIN_PI12 140
#define PIN_PI13 141
#define PIN_PI14 142
#define PIN_PI15 143
#define PIN_PJ0 144
#define PIN_PJ1 145
#define PIN_PJ2 146
#define PIN_PJ3 147
#define PIN_PJ4 148
#define PIN_PJ5 149
#define PIN_PJ6 150
#define PIN_PJ7 151
#define PIN_PJ8 152
#define PIN_PJ9 153
#define PIN_PJ10 154
#define PIN_PJ11 155
#define PIN_PJ12 156
#define PIN_PJ13 157
#define PIN_PJ14 158
#define PIN_PJ15 159
#define PIN_PK0 160
#define PIN_PK1 161
#define PIN_PK2 162
#define PIN_PK3 163
#define PIN_PK4 164
#define PIN_PK5 165
#define PIN_PK6 166
#define PIN_PK7 167
#define PIN_PK8 168
#define PIN_PK9 169
#define PIN_PK10 170
#define PIN_PK11 171
#define PIN_PK12 172
#define PIN_PK13 173
#define PIN_PK14 174
#define PIN_PK15 175

#define PIN_MAX PIN_PK15

__attribute__((unused)) static GPIO_TypeDef* PAL_GPIO_PORT[] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, 0, GPIOJ, GPIOK};

// Pin to port
#define PAL_GPIO_PORT(pin) PAL_GPIO_PORT[pin >> 4]

// Pin to index
#define PAL_GPIO_PIN(pin) (1 << (pin & 0xF))

// Pin aliases
#define PIN_FIREMAIN PIN_PE9
#define PIN_FIREDRG PIN_PA6
#define PIN_FIREA1 PIN_PA5
#define PIN_FIREA2 PIN_PC1
#define PIN_FIREA3 PIN_PD10
#define PIN_CONTMAIN PIN_PE8
#define PIN_CONTDRG PIN_PA2
#define PIN_CONTA1 PIN_PA7
#define PIN_CONTA2 PIN_PA4
#define PIN_CONTA3 PIN_PB15
#define PIN_RED PIN_PC10
#define PIN_YELLOW PIN_PC12
#define PIN_GREEN PIN_PD2
#define PIN_BLUE PIN_PD3
#define PIN_BUZZER PIN_PE5
#define PIN_PAUSE PIN_PB5

#endif  // BOARD_H