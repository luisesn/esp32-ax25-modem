#pragma once
#include <stdint.h>

typedef struct {
    volatile uint32_t frames_crc_ok;
    volatile uint32_t frames_crc_err;
    volatile uint32_t hdlc_flags;
    volatile uint32_t fifo_overflows;
} RxStats;

extern RxStats rx_stats_v1;
extern RxStats rx_stats_v2;

void rx_stats_reset(void);
