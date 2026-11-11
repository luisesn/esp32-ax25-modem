// Stub de la cola TX de AFSK para test_digipeater: captura cada trama emitida.
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "LibAPRS-esp32-i2s/src/AFSK.h"

#define STUB_TX_MAX 512
uint8_t stub_tx_buf[STUB_TX_MAX];
size_t  stub_tx_len   = 0;
int     stub_tx_count = 0;

void stub_tx_reset(void)
{
    memset(stub_tx_buf, 0, sizeof(stub_tx_buf));
    stub_tx_len = 0;
    stub_tx_count = 0;
}

bool afsk_queue_tx_frame(const uint8_t *data, size_t len)
{
    stub_tx_count++;
    stub_tx_len = len < STUB_TX_MAX ? len : STUB_TX_MAX;
    memcpy(stub_tx_buf, data, stub_tx_len);
    return true;
}

bool afsk_queue_tx_frame_wait(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    (void)timeout_ms;
    return afsk_queue_tx_frame(data, len);
}
