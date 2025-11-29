#ifndef WLCOMM_H
#define WLCOMM_H

#include "pspcom.h"
#include "status.h"

Status wlcomm_init();

Status wlcomm_recv_msg(pspcommsg* msg);
Status wlcomm_send_msg(pspcommsg* msg);

Status wlcomm_set_freq(uint32_t freq_hz);

#endif  // WLCOMM_H
