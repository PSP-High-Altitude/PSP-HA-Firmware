/*
    Board specific definitions such as pin assignments
*/

#ifndef BOARD_H
#define BOARD_H

#ifdef COMPAT_9K5
#include "board_9k5.h"
#else  // not COMPAT_9K5
#include "board_darkstar.h"
#endif

#endif  // BOARD_H