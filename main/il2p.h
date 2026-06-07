#pragma once
/*
 * IL2P — Improved Layer 2 Protocol
 *
 * Replaces AX.25/HDLC framing with Reed-Solomon FEC + LFSR scrambling.
 * Operates on the same Bell-202 AFSK modem (no RF changes).
 *
 * Config: { "il2p": { "enabled": false } }
 *
 * When enabled:
 *   RX: accepts IL2P frames in parallel with AX.25; delivers decoded frames
 *       to the same on_ax25_raw_frame callback as standard AX.25.
 *   TX: smart_tx_frame() routes by PID — PID=0xCC (IP) → IL2P, rest → AX.25.
 */

#include <stdint.h>
#include <stddef.h>
#include "LibAPRS-esp32-i2s/src/AX25.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize IL2P. Must be called after AFSK_init(). */
void il2p_init(bool enabled, ax25_raw_callback_t cb);

/* Feed one NRZI-decoded bit from each modem.
 * modem_id: 0 = v1 (AFSK_adc_isr), 1 = v2 (afsk_v2_process).
 * No-op when il2p is disabled. */
void il2p_rx_bit(uint8_t modem_id, uint8_t nrzi_bit);

/* Drain decoded frame queue and deliver to callback.
 * Call from aprs_poll_task, same cadence as APRS_poll(). */
void il2p_poll(void);

/* TX: encode AX.25 raw frame into IL2P and transmit.
 * Uses AFSK_transmit() with bit-stuffing disabled. */
void il2p_tx_frame(const uint8_t *ax25_raw, size_t len);

/* TX dispatcher: routes by PID byte in the AX.25 frame.
 *   PID == 0xCC (IP/RFC-1226) → il2p_tx_frame()
 *   PID != 0xCC              → APRS_send_raw_frame()
 * Register with afsk_set_tx_fn() when il2p.enabled=true. */
void smart_tx_frame(const uint8_t *ax25_raw, size_t len);

#ifdef __cplusplus
}
#endif
