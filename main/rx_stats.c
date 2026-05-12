#include "rx_stats.h"
#include <string.h>

RxStats rx_stats_v1;
RxStats rx_stats_v2;

void rx_stats_reset(void) {
    memset((void *)&rx_stats_v1, 0, sizeof(rx_stats_v1));
    memset((void *)&rx_stats_v2, 0, sizeof(rx_stats_v2));
}
