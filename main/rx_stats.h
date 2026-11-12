#pragma once
#include <stdint.h>

typedef struct {
    volatile uint32_t frames_crc_ok;
    volatile uint32_t frames_crc_err;
    volatile uint32_t hdlc_flags;
    volatile uint32_t fifo_overflows;
} RxStats;

extern RxStats rx_stats_v2;   // demodulador v2 (detector anterior, sólo con rx.dual_modem_enabled)
extern RxStats rx_stats_v3;   // demodulador v3 (correlador, el principal)

void rx_stats_reset(void);
