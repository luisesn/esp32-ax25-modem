/*
 * IL2P — Improved Layer 2 Protocol
 *
 * Frame structure (after NRZI decoding):
 *   N × 0x7E        preamble (not scrambled)
 *   0xF1 0x5E 0x48  sync word (24 bits, not scrambled)
 *   15 bytes        RS(15,13) encoded header  (scrambled)
 *   K × 255 bytes   RS(255,239) payload blocks (scrambled, last block ≤255)
 *
 * LFSR polynomial x^9 + x^4 + 1, initial state 0x1FF, reset per frame.
 * RS parity computed on UNSCRAMBLED data; data+parity scrambled on TX.
 * Receiver: descramble → RS decode.
 */

#include "il2p.h"
#include "rs_codec.h"
#include "AFSK.h"
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "il2p"

/* ─── public knobs ────────────────────────────────────────────────────────── */
static bool               s_enabled  = false;
static ax25_raw_callback_t s_cb       = NULL;

/* ─── protocol constants ─────────────────────────────────────────────────── */
#define IL2P_SYNC_WORD   0xF15E48UL   /* 24-bit, transmitted unscrambled      */
#define IL2P_HEADER_BYTES 15          /* 13 data + 2 RS parity                */
#define IL2P_HEADER_DATA  13
#define IL2P_RS_NROOTS    16          /* parity bytes per payload block       */
#define IL2P_BLOCK_DATA   239
#define IL2P_BLOCK_TOTAL  255
#define IL2P_MAX_PAYLOAD  1023
#define IL2P_PREAMBLE_MIN 4           /* minimum 0x7E bytes before sync       */

/* ─── LFSR scrambler ─────────────────────────────────────────────────────── */
static inline uint8_t lfsr_step(uint16_t *st)
{
    uint8_t out = (uint8_t)((*st >> 8) ^ (*st >> 3)) & 1;
    *st = (uint16_t)(((*st << 1) | out) & 0x1FF);
    return out;
}

static inline uint8_t il2p_descramble(uint16_t *st, uint8_t bit)
{
    return bit ^ lfsr_step(st);
}

/* ─── 6-bit callsign encoding (IL2P ↔ AX.25 ASCII) ─────────────────────── */
static uint8_t char_to_6bit(uint8_t c)
{
    if (c >= 'A' && c <= 'Z') return (uint8_t)(c - 'A' + 1);
    if (c >= 'a' && c <= 'z') return (uint8_t)(c - 'a' + 1);
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0' + 27);
    return 0; /* space / unknown */
}

static uint8_t char_from_6bit(uint8_t v)
{
    if (v == 0)       return ' ';
    if (v <= 26)      return (uint8_t)('A' + v - 1);
    if (v <= 36)      return (uint8_t)('0' + v - 27);
    return '?';
}

/* ─── Bit I/O helpers ────────────────────────────────────────────────────── */
static void pack_bits(uint8_t *buf, int *bit_pos, uint32_t val, int nbits)
{
    for (int i = nbits - 1; i >= 0; i--) {
        int byte_idx = *bit_pos / 8;
        int bit_idx  = 7 - (*bit_pos % 8);
        if ((val >> i) & 1) buf[byte_idx] |= (1 << bit_idx);
        else                buf[byte_idx] &= ~(1 << bit_idx);
        (*bit_pos)++;
    }
}

static uint32_t unpack_bits(const uint8_t *buf, int *bit_pos, int nbits)
{
    uint32_t val = 0;
    for (int i = 0; i < nbits; i++) {
        int byte_idx = *bit_pos / 8;
        int bit_idx  = 7 - (*bit_pos % 8);
        val = (val << 1) | ((buf[byte_idx] >> bit_idx) & 1);
        (*bit_pos)++;
    }
    return val;
}

/* ─── IL2P header encode/decode ──────────────────────────────────────────── */

/* Parse an AX.25 address field: returns pointer past end of addresses.
 * Fills dest_call/ssid, src_call/ssid (ASCII, NUL-terminated), and
 * *ctrl_offset (byte index of the control byte in frame). */
static const uint8_t *ax25_parse_addrs(const uint8_t *frame, size_t flen,
                                        char dest[7], int *dest_ssid,
                                        char src[7],  int *src_ssid,
                                        int *ctrl_offset)
{
    if (flen < 14) return NULL;

    for (int i = 0; i < 6; i++) dest[i] = (char)(frame[i] >> 1);
    dest[6]    = '\0';
    *dest_ssid = (frame[6] >> 1) & 0x0F;

    for (int i = 0; i < 6; i++) src[i] = (char)(frame[7 + i] >> 1);
    src[6]    = '\0';
    *src_ssid = (frame[13] >> 1) & 0x0F;

    /* End-of-address indicated by bit 0 of SSID byte */
    int offset = 14;
    /* Skip repeater addresses if present */
    while (offset >= 2 && !(frame[offset - 1] & 0x01) && (size_t)(offset + 6) < flen)
        offset += 7;

    *ctrl_offset = offset;
    return frame + offset;
}

/* Build 13-byte IL2P header from raw AX.25 frame.
 * Returns payload length, or -1 on error. */
static int il2p_encode_header(const uint8_t *ax25, size_t ax25_len,
                               uint8_t hdr[IL2P_HEADER_DATA])
{
    char dest[7], src[7];
    int dest_ssid, src_ssid, ctrl_off;

    if (!ax25_parse_addrs(ax25, ax25_len, dest, &dest_ssid, src, &src_ssid, &ctrl_off))
        return -1;
    if ((int)ax25_len < ctrl_off + 2) return -1; /* need at least ctrl + pid */

    uint8_t ctrl = ax25[ctrl_off];
    uint8_t pid  = ax25[ctrl_off + 1];
    int payload_len = (int)ax25_len - ctrl_off - 2;
    if (payload_len < 0) payload_len = 0;
    if (payload_len > IL2P_MAX_PAYLOAD) return -1;

    memset(hdr, 0, IL2P_HEADER_DATA);
    int bp = 0;

    /* Destination callsign: 6 × 6 bits */
    for (int i = 0; i < 6; i++)
        pack_bits(hdr, &bp, char_to_6bit((uint8_t)dest[i]), 6);

    /* Destination SSID: 4 bits */
    pack_bits(hdr, &bp, (uint32_t)dest_ssid, 4);

    /* Source callsign: 6 × 6 bits */
    for (int i = 0; i < 6; i++)
        pack_bits(hdr, &bp, char_to_6bit((uint8_t)src[i]), 6);

    /* Source SSID: 4 bits */
    pack_bits(hdr, &bp, (uint32_t)src_ssid, 4);

    /* Reserved + frame type: 4 bits (0=UI) */
    pack_bits(hdr, &bp, 0, 4);

    /* PID: 8 bits */
    pack_bits(hdr, &bp, pid, 8);

    /* Control: 8 bits */
    pack_bits(hdr, &bp, ctrl, 8);

    /* Payload length: 10 bits */
    pack_bits(hdr, &bp, (uint32_t)payload_len, 10);

    /* Padding bit (bp should be 104 = 13 bytes) */
    if (bp < 104) pack_bits(hdr, &bp, 0, 104 - bp);

    return payload_len;
}

/* Decode 13-byte IL2P header into AX.25 components.
 * Returns payload_len, or -1 on error. */
static int il2p_decode_header(const uint8_t hdr[IL2P_HEADER_DATA],
                               char dest[7], int *dest_ssid,
                               char src[7],  int *src_ssid,
                               uint8_t *ctrl, uint8_t *pid)
{
    int bp = 0;

    for (int i = 0; i < 6; i++) dest[i] = (char)char_from_6bit((uint8_t)unpack_bits(hdr, &bp, 6));
    dest[6]    = '\0';
    *dest_ssid = (int)unpack_bits(hdr, &bp, 4);

    for (int i = 0; i < 6; i++) src[i] = (char)char_from_6bit((uint8_t)unpack_bits(hdr, &bp, 6));
    src[6]    = '\0';
    *src_ssid  = (int)unpack_bits(hdr, &bp, 4);

    /* frame type (reserved, 4 bits) */
    unpack_bits(hdr, &bp, 4);

    *pid  = (uint8_t)unpack_bits(hdr, &bp, 8);
    *ctrl = (uint8_t)unpack_bits(hdr, &bp, 8);

    int payload_len = (int)unpack_bits(hdr, &bp, 10);
    return payload_len;
}

/* Reconstruct raw AX.25 frame (no CRC) from decoded header + payload.
 * Returns byte length of the AX.25 frame, or -1 if buf is too small. */
static int il2p_build_ax25(char dest[7], int dest_ssid,
                            char src[7],  int src_ssid,
                            uint8_t ctrl, uint8_t pid,
                            const uint8_t *payload, int payload_len,
                            uint8_t *out, size_t out_cap)
{
    /* Trim trailing spaces in callsigns */
    while (dest[0] && dest[strlen(dest)-1] == ' ')
        dest[strlen(dest)-1] = '\0';
    while (src[0]  && src[strlen(src)-1]  == ' ')
        src[strlen(src)-1]  = '\0';

    int total = 14 + 2 + payload_len; /* 2 addresses + ctrl + pid + info */
    if (total < 0 || (size_t)total > out_cap) return -1;

    /* Destination address */
    char tmp[7] = "      ";
    int clen = (int)strlen(dest);
    memcpy(tmp, dest, (size_t)(clen > 6 ? 6 : clen));
    for (int i = 0; i < 6; i++) out[i] = (uint8_t)((tmp[i] & 0x7F) << 1);
    out[6] = (uint8_t)(0x60 | ((dest_ssid & 0x0F) << 1)); /* H bit = 0 */

    /* Source address */
    clen = (int)strlen(src);
    memset(tmp, ' ', 6); tmp[6] = '\0';
    memcpy(tmp, src, (size_t)(clen > 6 ? 6 : clen));
    for (int i = 0; i < 6; i++) out[7 + i] = (uint8_t)((tmp[i] & 0x7F) << 1);
    out[13] = (uint8_t)(0x61 | ((src_ssid & 0x0F) << 1)); /* H bit = 1 (last addr) */

    out[14] = ctrl;
    out[15] = pid;
    if (payload_len > 0) memcpy(out + 16, payload, (size_t)payload_len);

    return total;
}

/* ─── RX state machine ───────────────────────────────────────────────────── */

typedef enum {
    IL2P_HUNT,    /* looking for 0x7E preamble  */
    IL2P_SYNC,    /* searching 24-bit sync word  */
    IL2P_HEADER,  /* collecting 15 header bytes  */
    IL2P_PAYLOAD, /* collecting payload RS blocks */
} il2p_state_t;

typedef struct {
    il2p_state_t state;
    uint32_t     sync_reg;       /* sliding 24-bit window              */
    int          preamble_count; /* 0x7E bytes seen so far             */
    uint16_t     lfsr;           /* 9-bit scrambler state              */

    /* bit accumulator → bytes */
    int          bit_count;
    uint8_t      cur_byte;

    /* header collection */
    uint8_t      hdr_raw[IL2P_HEADER_BYTES]; /* 15 RS-encoded bytes     */
    int          hdr_idx;

    /* payload collection */
    int          payload_len;    /* total payload bytes (from header)  */
    int          blocks_total;   /* RS blocks needed                   */
    int          block_idx;      /* current block (0-based)            */
    int          block_byte_idx; /* byte within current RS block       */
    int          block_data_len; /* data bytes in current block        */
    uint8_t      block_buf[IL2P_BLOCK_TOTAL]; /* one RS block buffer    */
    uint8_t      payload_buf[IL2P_MAX_PAYLOAD];
    int          payload_written;

    /* decoded frame staging */
    uint8_t      ax25_frame[AX25_MAX_FRAME_LEN];
    int          ax25_len;
    bool         frame_ready;
} il2p_rx_t;

static il2p_rx_t s_rx[2]; /* index 0 = modem v1, index 1 = modem v2 */

static void rx_reset(il2p_rx_t *rx)
{
    rx->state          = IL2P_HUNT;
    rx->sync_reg       = 0;
    rx->preamble_count = 0;
    rx->bit_count      = 0;
    rx->cur_byte       = 0;
    rx->hdr_idx        = 0;
    rx->frame_ready    = false;
}

static void rx_start_header(il2p_rx_t *rx)
{
    rx->state     = IL2P_HEADER;
    rx->lfsr      = 0x1FF; /* reset scrambler */
    rx->bit_count = 0;
    rx->hdr_idx   = 0;
}

static void rx_start_payload(il2p_rx_t *rx, int payload_len)
{
    rx->state           = IL2P_PAYLOAD;
    rx->payload_len     = payload_len;
    rx->blocks_total    = (payload_len + IL2P_BLOCK_DATA - 1) / IL2P_BLOCK_DATA;
    rx->block_idx       = 0;
    rx->block_byte_idx  = 0;
    rx->payload_written = 0;
    rx->bit_count       = 0;

    if (rx->blocks_total == 0) {
        /* No payload — frame is already complete */
        rx->frame_ready = true;
        rx->state       = IL2P_HUNT;
        return;
    }

    /* First block data length */
    if (rx->blocks_total == 1)
        rx->block_data_len = payload_len;
    else
        rx->block_data_len = IL2P_BLOCK_DATA;
    memset(rx->block_buf, 0, sizeof(rx->block_buf));
}

/* Process one descrambled byte in HEADER phase */
static void rx_header_byte(il2p_rx_t *rx, uint8_t b)
{
    rx->hdr_raw[rx->hdr_idx++] = b;
    if (rx->hdr_idx < IL2P_HEADER_BYTES) return;

    /* All 15 header bytes received — RS decode */
    uint8_t tmp[IL2P_HEADER_BYTES];
    memcpy(tmp, rx->hdr_raw, IL2P_HEADER_BYTES);
    int res = rs4_decode(tmp);
    if (res < 0) {
        ESP_LOGD(TAG, "Header RS decode fail");
        rx_reset(rx);
        return;
    }

    /* Decode header fields */
    char dest[7], src[7];
    int  dest_ssid, src_ssid, payload_len;
    uint8_t ctrl, pid;
    payload_len = il2p_decode_header(tmp, dest, &dest_ssid, src, &src_ssid, &ctrl, &pid);
    if (payload_len < 0 || payload_len > IL2P_MAX_PAYLOAD) {
        rx_reset(rx);
        return;
    }

    /* Pre-build AX.25 frame header (addresses + ctrl + pid) */
    int n = il2p_build_ax25(dest, dest_ssid, src, src_ssid,
                             ctrl, pid, NULL, 0,
                             rx->ax25_frame, sizeof(rx->ax25_frame));
    if (n < 0) { rx_reset(rx); return; }
    rx->ax25_len = n;

    if (payload_len == 0) {
        rx->frame_ready = true;
        rx->state       = IL2P_HUNT;
    } else {
        rx_start_payload(rx, payload_len);
    }
}

/* Process one descrambled byte in PAYLOAD phase */
static void rx_payload_byte(il2p_rx_t *rx, uint8_t b)
{
    rx->block_buf[rx->block_byte_idx++] = b;

    int block_total_bytes = rx->block_data_len + IL2P_RS_NROOTS;
    if (rx->block_byte_idx < block_total_bytes) return;

    /* Full RS block received — decode */
    int res = rs8_decode(rx->block_buf, rx->block_data_len);
    if (res < 0) {
        ESP_LOGD(TAG, "Payload RS decode fail block %d", rx->block_idx);
        rx_reset(rx);
        return;
    }

    /* Append decoded data to payload buffer */
    int avail = (int)sizeof(rx->payload_buf) - rx->payload_written;
    int copy  = (rx->block_data_len < avail) ? rx->block_data_len : avail;
    memcpy(rx->payload_buf + rx->payload_written, rx->block_buf, (size_t)copy);
    rx->payload_written += copy;

    rx->block_idx++;
    if (rx->block_idx >= rx->blocks_total) {
        /* All blocks decoded — append payload to AX.25 frame */
        int remaining = (int)sizeof(rx->ax25_frame) - rx->ax25_len;
        copy = (rx->payload_written < remaining) ? rx->payload_written : remaining;
        memcpy(rx->ax25_frame + rx->ax25_len, rx->payload_buf, (size_t)copy);
        rx->ax25_len  += copy;
        rx->frame_ready = true;
        rx->state       = IL2P_HUNT;
        return;
    }

    /* Prepare next block */
    rx->block_byte_idx = 0;
    memset(rx->block_buf, 0, sizeof(rx->block_buf));
    int remaining_data = rx->payload_len - rx->payload_written;
    rx->block_data_len = (remaining_data < IL2P_BLOCK_DATA) ? remaining_data : IL2P_BLOCK_DATA;
}

/* Feed one NRZI-decoded bit to the RX state machine */
static void rx_process_bit(il2p_rx_t *rx, uint8_t bit)
{
    switch (rx->state) {
    case IL2P_HUNT:
        /* Assemble bytes and count 0x7E flags */
        rx->sync_reg = ((rx->sync_reg << 1) | (bit & 1)) & 0xFF;
        rx->bit_count++;
        if (rx->bit_count < 8) break;
        rx->bit_count = 0;
        if (rx->sync_reg == 0x7E) {
            rx->preamble_count++;
        } else {
            rx->preamble_count = 0;
        }
        rx->sync_reg = 0;
        /* Switch to SYNC search after ≥ IL2P_PREAMBLE_MIN flags */
        if (rx->preamble_count >= IL2P_PREAMBLE_MIN) {
            rx->state     = IL2P_SYNC;
            rx->sync_reg  = 0;
            rx->bit_count = 0;
        }
        break;

    case IL2P_SYNC:
        rx->sync_reg = ((rx->sync_reg << 1) | (bit & 1)) & 0xFFFFFF;
        if (rx->sync_reg == IL2P_SYNC_WORD) {
            rx_start_header(rx);
        }
        /* Timeout: fall back to HUNT if sync word not found after ~64 extra bytes */
        rx->bit_count++;
        if (rx->bit_count > 512) {
            rx_reset(rx);
        }
        break;

    case IL2P_HEADER:
    case IL2P_PAYLOAD: {
        /* Descramble the bit */
        uint8_t dbit = il2p_descramble(&rx->lfsr, bit);
        /* Accumulate into byte */
        rx->cur_byte = (uint8_t)((rx->cur_byte << 1) | dbit);
        rx->bit_count++;
        if (rx->bit_count < 8) break;
        rx->bit_count = 0;
        uint8_t byte = rx->cur_byte;
        rx->cur_byte = 0;
        if (rx->state == IL2P_HEADER)
            rx_header_byte(rx, byte);
        else
            rx_payload_byte(rx, byte);
        break;
    }
    }
}

/* ─── Deduplication ──────────────────────────────────────────────────────── */

#define DEDUP_SLOTS 8
#define DEDUP_WINDOW_MS 500

typedef struct { uint16_t hash; int64_t ts_us; } dedup_entry_t;
static dedup_entry_t s_dedup[DEDUP_SLOTS];
static int           s_dedup_head = 0;

static uint16_t frame_hash(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static bool dedup_seen(const uint8_t *buf, size_t len)
{
    uint16_t h = frame_hash(buf, len);
    int64_t  now = esp_timer_get_time();
    for (int i = 0; i < DEDUP_SLOTS; i++) {
        if (s_dedup[i].hash == h &&
            (now - s_dedup[i].ts_us) < (int64_t)(DEDUP_WINDOW_MS * 1000))
            return true;
    }
    /* Record */
    s_dedup[s_dedup_head].hash  = h;
    s_dedup[s_dedup_head].ts_us = now;
    s_dedup_head = (s_dedup_head + 1) % DEDUP_SLOTS;
    return false;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

void il2p_init(bool enabled, ax25_raw_callback_t cb)
{
    s_enabled = enabled;
    s_cb      = cb;
    memset(s_rx,    0, sizeof(s_rx));
    memset(s_dedup, 0, sizeof(s_dedup));
    for (int i = 0; i < 2; i++) rx_reset(&s_rx[i]);
    if (enabled)
        ESP_LOGI(TAG, "IL2P enabled — RX dual-modem, TX smart routing by PID");
}

void il2p_rx_bit(uint8_t modem_id, uint8_t nrzi_bit)
{
    if (!s_enabled || modem_id > 1) return;
    rx_process_bit(&s_rx[modem_id], nrzi_bit & 1);
}

void il2p_poll(void)
{
    if (!s_enabled || !s_cb) return;
    for (int m = 0; m < 2; m++) {
        il2p_rx_t *rx = &s_rx[m];
        if (!rx->frame_ready) continue;
        rx->frame_ready = false;
        if (rx->ax25_len < 16) continue; /* too short */
        if (!dedup_seen((const uint8_t *)rx->ax25_frame, (size_t)rx->ax25_len)) {
            ESP_LOGD(TAG, "IL2P frame %d bytes (modem v%d)", rx->ax25_len, m + 1);
            s_cb((const uint8_t *)rx->ax25_frame, (size_t)rx->ax25_len);
        }
    }
}

/* ─── TX ─────────────────────────────────────────────────────────────────── */

/* Max IL2P frame: preamble(16) + sync(3) + header(15) + payload_blocks(4×255) = ~1054 */
#define IL2P_TX_MAX (16 + 3 + IL2P_HEADER_BYTES + (4 * IL2P_BLOCK_TOTAL) + 32)

void il2p_tx_frame(const uint8_t *ax25_raw, size_t len)
{
    if (!s_enabled) return;

    static uint8_t outbuf[IL2P_TX_MAX];
    int out_idx = 0;

    /* 1. Preamble: 16 × 0x7E */
    for (int i = 0; i < 16; i++) outbuf[out_idx++] = 0x7E;

    /* 2. Sync word */
    outbuf[out_idx++] = 0xF1;
    outbuf[out_idx++] = 0x5E;
    outbuf[out_idx++] = 0x48;

    /* 3. Header */
    uint8_t hdr_data[IL2P_HEADER_DATA];
    int payload_len = il2p_encode_header(ax25_raw, len, hdr_data);
    if (payload_len < 0) {
        ESP_LOGW(TAG, "il2p_tx: header encode failed");
        return;
    }

    uint8_t hdr_parity[2];
    rs4_encode(hdr_data, hdr_parity);

    /* Scramble header (data + parity) */
    uint16_t lfsr = 0x1FF;
    for (int i = 0; i < IL2P_HEADER_DATA; i++) {
        uint8_t b = hdr_data[i], sb = 0;
        for (int bit = 7; bit >= 0; bit--)
            sb |= (uint8_t)(il2p_descramble(&lfsr, (b >> bit) & 1) << bit);
        outbuf[out_idx++] = sb;
    }
    for (int i = 0; i < 2; i++) {
        uint8_t b = hdr_parity[i], sb = 0;
        for (int bit = 7; bit >= 0; bit--)
            sb |= (uint8_t)(il2p_descramble(&lfsr, (b >> bit) & 1) << bit);
        outbuf[out_idx++] = sb;
    }

    /* 4. Payload blocks */
    /* Extract payload from AX.25 frame: skip 2 address fields (14 bytes) + ctrl + pid */
    char dest[7], src[7];
    int  dest_ssid, src_ssid, ctrl_off;
    if (!ax25_parse_addrs(ax25_raw, len, dest, &dest_ssid, src, &src_ssid, &ctrl_off)) {
        ESP_LOGW(TAG, "il2p_tx: bad AX.25 addresses");
        return;
    }
    const uint8_t *payload_ptr = ax25_raw + ctrl_off + 2;
    int            payload_left = payload_len;

    while (payload_left > 0) {
        int block_data = (payload_left < IL2P_BLOCK_DATA) ? payload_left : IL2P_BLOCK_DATA;
        uint8_t parity[IL2P_RS_NROOTS];
        rs8_encode(payload_ptr, block_data, parity);

        /* Scramble data bytes */
        for (int i = 0; i < block_data; i++) {
            uint8_t b = payload_ptr[i], sb = 0;
            for (int bit = 7; bit >= 0; bit--)
                sb |= (uint8_t)(il2p_descramble(&lfsr, (b >> bit) & 1) << bit);
            if (out_idx < IL2P_TX_MAX) outbuf[out_idx++] = sb;
        }
        /* Scramble parity bytes */
        for (int i = 0; i < IL2P_RS_NROOTS; i++) {
            uint8_t b = parity[i], sb = 0;
            for (int bit = 7; bit >= 0; bit--)
                sb |= (uint8_t)(il2p_descramble(&lfsr, (b >> bit) & 1) << bit);
            if (out_idx < IL2P_TX_MAX) outbuf[out_idx++] = sb;
        }

        payload_ptr  += block_data;
        payload_left -= block_data;
    }

    /* 5. Transmit — bit-stuffing must be disabled */
    extern Afsk *AFSK_modem;
    if (AFSK_modem) AFSK_modem->il2p_tx = true;
    AFSK_transmit((char *)outbuf, (size_t)out_idx);
    if (AFSK_modem) AFSK_modem->il2p_tx = false;
}

void smart_tx_frame(const uint8_t *ax25_raw, size_t len)
{
    if (!s_enabled) {
        /* Fallback: should not be called when disabled, but be safe */
        extern void APRS_send_raw_frame(const uint8_t *buf, size_t len);
        APRS_send_raw_frame(ax25_raw, len);
        return;
    }

    /* Find PID byte: skip 2 address fields (14 bytes min) + optional repeaters + control */
    char dest[7], src[7];
    int  dest_ssid, src_ssid, ctrl_off;
    if (!ax25_parse_addrs(ax25_raw, len, dest, &dest_ssid, src, &src_ssid, &ctrl_off) ||
        (int)len < ctrl_off + 2) {
        /* Malformed frame — fall back to AX.25 */
        extern void APRS_send_raw_frame(const uint8_t *buf, size_t len);
        APRS_send_raw_frame(ax25_raw, len);
        return;
    }

    uint8_t pid = ax25_raw[ctrl_off + 1];
    if (pid == 0xCC) {
        il2p_tx_frame(ax25_raw, len);
    } else {
        extern void APRS_send_raw_frame(const uint8_t *buf, size_t len);
        APRS_send_raw_frame(ax25_raw, len);
    }
}
