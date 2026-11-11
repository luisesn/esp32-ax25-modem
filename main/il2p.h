#pragma once
/*
 * IL2P — Improved Layer 2 Protocol
 *
 * Replaces AX.25/HDLC framing with Reed-Solomon FEC + LFSR scrambling.
 * Operates on the same Bell-202 AFSK modem (no RF changes).
 *
 * Config: { "il2p": { "enabled": false, "max_fec": true, "tx_all": false } }
 * Formato estándar (compatible con Dire Wolf): cabecera 13+2 tipo 0/1, sync
 * 0xF15E48, RS GF(2^8) fcr=0, scrambler por bloque, bits MSB primero.
 *
 * When enabled:
 *   RX: accepts IL2P frames in parallel with AX.25; delivers decoded frames
 *       to the same on_ax25_raw_frame callback as standard AX.25.
 *   TX: smart_tx_frame() routes by PID — PID=0xCC (IP) → IL2P, rest → AX.25
 *       (con tx_all=true todo va por IL2P). Tramas TUN crudas: il2p_tx_raw().
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

/* TX de bytes opacos (tramas TUN 00 00 08 00 + IPv4) con cabecera tipo 0. */
void il2p_tx_raw(const uint8_t *data, size_t len);

/* Opciones de TX: max_fec = 16 símbolos de paridad por bloque; tx_all = todo
 * el tráfico por IL2P (no sólo PID 0xCC). */
void il2p_set_options(bool max_fec, bool tx_all);

/* Codificación/decodificación de una trama completa sin sync (pruebas en host).
 * out ≥ 1100 B en decode, ≥ 1100+80 en encode. Devuelven longitud o -1. */
int il2p_encode_frame(const uint8_t *ax, size_t len, bool raw, bool max_fec, uint8_t *out);
int il2p_decode_frame(const uint8_t *in, uint8_t *out);
void il2p_scramble_block(const uint8_t *in, uint8_t *out, int len);
void il2p_descramble_block(const uint8_t *in, uint8_t *out, int len);

/* TX dispatcher: routes by PID byte in the AX.25 frame.
 *   PID == 0xCC (IP/RFC-1226) → il2p_tx_frame()
 *   PID != 0xCC              → APRS_send_raw_frame()
 * Register with afsk_set_tx_fn() when il2p.enabled=true. */
void smart_tx_frame(const uint8_t *ax25_raw, size_t len);

#ifdef __cplusplus
}
#endif
