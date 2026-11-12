#include "rx_stats.h"
#include <string.h>

RxStats rx_stats_v3;
RxStats rx_stats_v2;

void rx_stats_reset(void) {
    memset((void *)&rx_stats_v3, 0, sizeof(rx_stats_v3));
    memset((void *)&rx_stats_v2, 0, sizeof(rx_stats_v2));
}
