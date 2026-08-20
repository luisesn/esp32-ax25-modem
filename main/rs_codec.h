#pragma once
/*
 * Reed-Solomon codec for IL2P
 *
 * Two parameter sets used by IL2P (Improved Layer 2 Protocol):
 *   RS(255,239) over GF(2^8)         — payload blocks, 16 parity bytes, corrects ≤8 errors
 *   Shortened RS over GF(2^8), 2 roots — header, 2 parity bytes, corrects ≤1 error
 *
 * The header FEC shares the GF(2^8) field with the payload codec (not a
 * separate GF(2^4)): the header holds raw 8-bit bytes (packed callsigns,
 * PID, control), and GF(2^4) symbols can only represent values 0-15.
 *
 * Adapted from Phil Karn's libfec (LGPL) / Direwolf by WB2OSZ.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── RS(255,239) over GF(2^8) ──────────────────────────────────────────────
 * dlen: actual data length (1 … 239).  Shortening handled internally.
 * parity: output buffer of exactly 16 bytes.
 */
void rs8_encode(const uint8_t *data, int dlen, uint8_t *parity);

/* block = data (dlen bytes) followed by parity (16 bytes), total dlen+16.
 * Corrects up to 8 symbol errors in place.
 * Returns number of corrections made, or -1 if uncorrectable.
 */
int  rs8_decode(uint8_t *block, int dlen);

/* ── Shortened RS over GF(2^8), 2 parity bytes ─────────────────────────────
 * dlen: actual header data length in bytes.  parity: output buffer of
 * exactly 2 bytes.
 */
void rsh_encode(const uint8_t *data, int dlen, uint8_t *parity);

/* block = data (dlen bytes) followed by parity (2 bytes), total dlen+2.
 * Corrects up to 1 symbol error in place.
 * Returns 0 (no error), 1 (corrected), or -1 (uncorrectable).
 */
int  rsh_decode(uint8_t *block, int dlen);

#ifdef __cplusplus
}
#endif
