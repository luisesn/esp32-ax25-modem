#pragma once
/*
 * Reed-Solomon codec for IL2P
 *
 * Two parameter sets used by IL2P (Improved Layer 2 Protocol):
 *   RS(255,239) over GF(2^8)  — payload blocks, 16 parity bytes, corrects ≤8 errors
 *   RS(15,13)   over GF(2^4)  — 13-byte header, 2 parity bytes, corrects ≤1 error
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

/* ── RS(15,13) over GF(2^4) ────────────────────────────────────────────────
 * data: exactly 13 bytes.  parity: output buffer of exactly 2 bytes.
 */
void rs4_encode(const uint8_t *data, uint8_t *parity);

/* block: exactly 15 bytes (13 data + 2 parity), corrected in place.
 * Returns 0 (no error), 1 (corrected), or -1 (uncorrectable).
 */
int  rs4_decode(uint8_t *block);

#ifdef __cplusplus
}
#endif
