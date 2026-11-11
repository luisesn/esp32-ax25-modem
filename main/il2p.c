/*
 * IL2P — Improved Layer 2 Protocol (compatible con Dire Wolf / spec TARPN).
 *
 * Trama (bits MSB primero, tras NRZI):
 *   preámbulo 0x55…, sync 0xF1 0x5E 0x48 (sin scrambler),
 *   cabecera 13 B + 2 B paridad RS (scrambler + RS(2 raíces)),
 *   carga útil en bloques "pequeños/grandes", cada uno scrambler + RS
 *   (2/4/6/8 raíces automáticas, o 16 con max_fec).
 *
 * Cabecera tipo 1: direcciones en SIXBIT (' '..'_'), sólo 2 direcciones
 *   (sin repetidores) y tramas AX.25 mod-8; la carga es el campo de info.
 * Cabecera tipo 0: encapsulado transparente; la carga es la trama completa
 *   (cualquier otra cosa, y las tramas TUN "crudas" 00 00 08 00 + IPv4 de
 *   ax25ip, que sólo entienden equipos con este firmware).
 *
 * El RS usa GF(2^8) 0x11D, fcr=0 (rs_codec.c). Scrambler: LFSR x^9+x^4+1
 * por bloque (reinicia en cada cabecera/bloque), con el truco de Dire Wolf
 * de descartar 5 bits al inicio y añadir 5 al final.
 */

#include "il2p.h"
#include "dedup.h"
#include "rs_codec.h"
#include "AFSK.h"
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"

#define TAG "il2p"

static bool                s_enabled = false;
static bool                s_max_fec = true;
static bool                s_tx_all  = false;
static ax25_raw_callback_t s_cb      = NULL;

#define IL2P_SYNC_WORD   0xF15E48UL
#define IL2P_HDR_SIZE    13
#define IL2P_HDR_PARITY  2
#define IL2P_MAX_PAYLOAD 1023
#define IL2P_MAX_BLOCKS  5
#define IL2P_MAX_ENC     (IL2P_MAX_PAYLOAD + IL2P_MAX_BLOCKS * 16)
#define IL2P_PREAMBLE_N  4
#define IL2P_TX_MAX      (IL2P_PREAMBLE_N + 3 + IL2P_HDR_SIZE + IL2P_HDR_PARITY + IL2P_MAX_ENC)
#define FRAME_BUF_MAX    (IL2P_MAX_PAYLOAD + 20)

void il2p_set_options(bool max_fec, bool tx_all)
{
    s_max_fec = max_fec;
    s_tx_all  = tx_all;
}

/* ─── Scrambler (Dire Wolf il2p_scramble.c) ─────────────────────────────── */

#define INIT_TX_LFSR 0x00f
#define INIT_RX_LFSR 0x1f0

static inline int scramble_bit(int in, int *state)
{
    int out = ((*state >> 4) ^ *state) & 1;
    *state = ((((in ^ *state) & 1) << 9) | (*state ^ ((*state & 1) << 4))) >> 1;
    return out;
}

static inline int descramble_bit(int in, int *state)
{
    int out = (in ^ *state) & 1;
    *state = ((*state >> 1) | ((in & 1) << 8)) ^ ((in & 1) << 3);
    return out;
}

void il2p_scramble_block(const uint8_t *in, uint8_t *out, int len)
{
    int st = INIT_TX_LFSR;
    memset(out, 0, (size_t)len);
    int skipping = 1, ob = 0, om = 0x80;
    for (int ib = 0; ib < len; ib++) {
        for (int im = 0x80; im != 0; im >>= 1) {
            int s = scramble_bit((in[ib] & im) != 0, &st);
            if (ib == 0 && im == 0x04) skipping = 0;
            if (!skipping) {
                if (s) out[ob] |= (uint8_t)om;
                om >>= 1;
                if (om == 0) { om = 0x80; ob++; }
            }
        }
    }
    int x = st;
    for (int n = 0; n < 5; n++) {
        int s = scramble_bit(0, &x);
        if (s) out[ob] |= (uint8_t)om;
        om >>= 1;
        if (om == 0) { om = 0x80; ob++; }
    }
}

void il2p_descramble_block(const uint8_t *in, uint8_t *out, int len)
{
    int st = INIT_RX_LFSR;
    memset(out, 0, (size_t)len);
    for (int b = 0; b < len; b++)
        for (int m = 0x80; m != 0; m >>= 1)
            if (descramble_bit((in[b] & m) != 0, &st)) out[b] |= (uint8_t)m;
}

/* ─── Bloques de carga útil ─────────────────────────────────────────────── */

typedef struct {
    int count, small_size, large_size, small_count, large_count, parity;
} payload_props_t;

/* Devuelve nº de bytes codificados (0 sin carga), -1 si el tamaño es inválido. */
static int payload_compute(payload_props_t *p, int size, int max_fec)
{
    memset(p, 0, sizeof(*p));
    if (size < 0 || size > IL2P_MAX_PAYLOAD) return -1;
    if (size == 0) return 0;
    p->count = (size + (max_fec ? 238 : 246)) / (max_fec ? 239 : 247);
    p->small_size  = size / p->count;
    p->large_size  = p->small_size + 1;
    p->large_count = size - p->count * p->small_size;
    p->small_count = p->count - p->large_count;
    if (max_fec)                       p->parity = 16;
    else if (p->small_size <= 61)      p->parity = 2;
    else if (p->small_size <= 123)     p->parity = 4;
    else if (p->small_size <= 185)     p->parity = 6;
    else if (p->small_size <= 247)     p->parity = 8;
    else return -1;
    return p->small_count * (p->small_size + p->parity) +
           p->large_count * (p->large_size + p->parity);
}

static int encode_payload(const uint8_t *in, int size, int max_fec, uint8_t *out)
{
    payload_props_t p;
    int e = payload_compute(&p, size, max_fec);
    if (e <= 0) return e;
    uint8_t scram[256];
    int olen = 0;
    for (int pass = 0; pass < 2; pass++) {
        int cnt = pass == 0 ? p.large_count : p.small_count;
        int sz  = pass == 0 ? p.large_size  : p.small_size;
        for (int b = 0; b < cnt; b++) {
            il2p_scramble_block(in, scram, sz);
            memcpy(out + olen, scram, (size_t)sz);
            il2p_rs_encode(scram, sz, p.parity, out + olen + sz);
            in   += sz;
            olen += sz + p.parity;
        }
    }
    return olen;
}

/* Devuelve bytes decodificados o <0 si algún bloque no se recupera. */
static int decode_payload(const uint8_t *enc, int size, int max_fec, uint8_t *out)
{
    payload_props_t p;
    int e = payload_compute(&p, size, max_fec);
    if (e <= 0) return e;
    uint8_t blk[255];
    int done = 0;
    for (int pass = 0; pass < 2; pass++) {
        int cnt = pass == 0 ? p.large_count : p.small_count;
        int sz  = pass == 0 ? p.large_size  : p.small_size;
        for (int b = 0; b < cnt; b++) {
            memcpy(blk, enc, (size_t)(sz + p.parity));
            if (il2p_rs_decode(blk, sz, p.parity) < 0) return -2;
            il2p_descramble_block(blk, out + done, sz);
            enc  += sz + p.parity;
            done += sz;
        }
    }
    return done;
}

/* ─── Cabecera ──────────────────────────────────────────────────────────── */

static void set_field(uint8_t *h, int bit, int lsb_idx, int width, int val)
{
    while (width > 0 && val != 0) {
        if (val & 1) h[lsb_idx] |= (uint8_t)(1 << bit);
        val >>= 1; lsb_idx--; width--;
    }
}

static int get_field(const uint8_t *h, int bit, int lsb_idx, int width)
{
    int r = 0;
    lsb_idx -= width - 1;
    while (width-- > 0) {
        r <<= 1;
        if (h[lsb_idx] & (1 << bit)) r |= 1;
        lsb_idx++;
    }
    return r;
}

#define SET_UI(h, v)       set_field(h, 6, 0, 1, v)
#define SET_PID(h, v)      set_field(h, 6, 4, 4, v)
#define SET_CONTROL(h, v)  set_field(h, 6, 11, 7, v)
#define SET_FEC(h, v)      set_field(h, 7, 0, 1, v)
#define SET_TYPE(h, v)     set_field(h, 7, 1, 1, v)
#define SET_COUNT(h, v)    set_field(h, 7, 11, 10, v)
#define GET_UI(h)          get_field(h, 6, 0, 1)
#define GET_PID(h)         get_field(h, 6, 4, 4)
#define GET_CONTROL(h)     get_field(h, 6, 11, 7)
#define GET_FEC(h)         get_field(h, 7, 0, 1)
#define GET_TYPE(h)        get_field(h, 7, 1, 1)
#define GET_COUNT(h)       get_field(h, 7, 11, 10)

static int encode_pid(uint8_t pid)
{
    if ((pid & 0x30) == 0x20 || (pid & 0x30) == 0x10) return 0x2;
    switch (pid) {
    case 0x01: return 0x3;
    case 0x06: return 0x4;
    case 0x07: return 0x5;
    case 0x08: return 0x6;
    case 0xcc: return 0xb;
    case 0xcd: return 0xc;
    case 0xce: return 0xd;
    case 0xcf: return 0xe;
    case 0xf0: return 0xf;
    }
    return -1;
}

static uint8_t decode_pid(int p)
{
    static const uint8_t t[16] = {0xf0, 0xf0, 0x20, 0x01, 0x06, 0x07, 0x08, 0xf0,
                                  0xf0, 0xf0, 0xf0, 0xcc, 0xcd, 0xce, 0xcf, 0xf0};
    return t[p & 15];
}

/* Códigos de control U (control & 0xEF) en el orden de la tabla IL2P (0..7). */
static const uint8_t U_CODES[8] = {0x2F, 0x43, 0x0F, 0x63, 0x87, 0x03, 0xAF, 0xE3};

/* Intenta cabecera tipo 1. Devuelve longitud de info (≥0), -1 si hay que usar
 * tipo 0, -2 si la info es demasiado grande. *info apunta dentro de ax. */
static int type1_header(const uint8_t *ax, size_t len, int max_fec,
                        uint8_t hdr[IL2P_HDR_SIZE], const uint8_t **info)
{
    memset(hdr, 0, IL2P_HDR_SIZE);
    if (len < 15 || (ax[13] & 1) == 0) return -1;   /* con repetidores → tipo 0 */

    for (int i = 0; i < 6; i++) {
        uint8_t d = ax[i] >> 1, s = ax[7 + i] >> 1;
        if (d < ' ' || d > '_' || s < ' ' || s > '_') return -1;
        hdr[i]     = (uint8_t)(d - ' ');
        hdr[6 + i] = (uint8_t)(s - ' ');
    }
    hdr[12] = (uint8_t)((((ax[6] >> 1) & 0xF) << 4) | ((ax[13] >> 1) & 0xF));

    uint8_t c = ax[14];
    int cmd = (ax[6] & 0x80) ? 1 : 0;
    int pf = (c >> 4) & 1, off;

    if ((c & 1) == 0) {                                   /* I */
        if (len < 16) return -1;
        int pid = encode_pid(ax[15]);
        if (pid < 0) return -1;
        SET_UI(hdr, 0);
        SET_PID(hdr, pid);
        SET_CONTROL(hdr, (pf << 6) | (((c >> 5) & 7) << 3) | ((c >> 1) & 7));
        off = 16;
    } else if ((c & 3) == 1) {                            /* S */
        SET_UI(hdr, 0);
        SET_PID(hdr, 0);
        SET_CONTROL(hdr, (pf << 6) | (((c >> 5) & 7) << 3) | (cmd << 2) | ((c >> 2) & 3));
        off = 15;
    } else {                                              /* U */
        int idx = -1;
        for (int i = 0; i < 8; i++) if ((c & 0xEF) == U_CODES[i]) idx = i;
        if (idx < 0) return -1;                           /* SABME, desconocidas */
        if (idx == 5) {                                   /* UI */
            if (len < 16) return -1;
            int pid = encode_pid(ax[15]);
            if (pid < 0) return -1;
            SET_UI(hdr, 1);
            SET_PID(hdr, pid);
            off = 16;
        } else {
            SET_PID(hdr, 1);
            off = 15;
        }
        SET_CONTROL(hdr, (pf << 6) | (cmd << 2) | (idx << 3));
    }
    SET_FEC(hdr, max_fec);
    SET_TYPE(hdr, 1);
    int ilen = (int)len - off;
    if (ilen > IL2P_MAX_PAYLOAD) return -2;
    SET_COUNT(hdr, ilen);
    *info = ax + off;
    return ilen;
}

static void type0_header(int frame_len, int max_fec, uint8_t hdr[IL2P_HDR_SIZE])
{
    memset(hdr, 0, IL2P_HDR_SIZE);
    SET_FEC(hdr, max_fec);
    SET_TYPE(hdr, 0);
    SET_COUNT(hdr, frame_len);
}

/* Reconstruye el principio de la trama AX.25 (direcciones + control [+ pid])
 * a partir de una cabecera tipo 1. Devuelve su longitud o -1 si no es válida. */
static int type1_to_ax25(const uint8_t *hdr, uint8_t *out)
{
    for (int a = 0; a < 2; a++) {
        const uint8_t *src = hdr + 6 * a;
        int last = 5;
        while (last >= 0 && (src[last] & 0x3f) == 0) last--;   /* quitar espacios finales */
        if (last < 0) return -1;
        for (int i = 0; i < 6; i++) {
            int ch = (i <= last) ? ((src[i] & 0x3f) + ' ') : ' ';
            if (i <= last && !((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9')))
                return -1;
            out[7 * a + i] = (uint8_t)(ch << 1);
        }
    }
    int dssid = (hdr[12] >> 4) & 0xF, sssid = hdr[12] & 0xF;
    int pid = GET_PID(hdr), ui = GET_UI(hdr), ctl = GET_CONTROL(hdr);
    int pf = (ctl >> 6) & 1, cmd, c, n;

    if (pid == 0) {                                       /* S */
        cmd = (ctl & 4) != 0;
        c = (((ctl >> 3) & 7) << 5) | (pf << 4) | ((ctl & 3) << 2) | 1;
        n = 15;
    } else if (pid == 1) {                                /* U (no UI) */
        cmd = (ctl & 4) != 0;
        int idx = (ctl >> 3) & 7;
        c = (pf << 4) | (U_CODES[idx] & 0xEF);
        if (idx == 5) { out[15] = 0xF0; n = 16; } else n = 15;
    } else if (ui) {                                      /* UI */
        cmd = (ctl & 4) != 0;
        c = (pf << 4) | 0x03;
        out[15] = decode_pid(pid);
        n = 16;
    } else {                                              /* I */
        cmd = 1;
        c = (((ctl >> 3) & 7) << 5) | (pf << 4) | ((ctl & 7) << 1);
        out[15] = decode_pid(pid);
        n = 16;
    }
    out[6]  = (uint8_t)(0x60 | (dssid << 1) | (cmd ? 0x80 : 0));
    out[13] = (uint8_t)(0x61 | (sssid << 1) | (cmd ? 0 : 0x80));
    out[14] = (uint8_t)c;
    return n;
}

/* ─── Codificación / decodificación de trama completa (sin sync) ────────── */

/* Codifica una trama AX.25 (sin FCS) — o, con raw=true, unos bytes opacos con
 * cabecera tipo 0 — a cabecera+paridad+bloques. Devuelve longitud o -1. */
int il2p_encode_frame(const uint8_t *ax, size_t len, bool raw, bool max_fec, uint8_t *out)
{
    uint8_t hdr[IL2P_HDR_SIZE];
    const uint8_t *pl = NULL;
    int pl_len = 0;
    int mf = max_fec ? 1 : 0;

    int e = raw ? -1 : type1_header(ax, len, mf, hdr, &pl);
    if (e >= 0) {
        pl_len = e;
    } else if (e == -1) {
        if (len < (raw ? 1u : 14u) || len > IL2P_MAX_PAYLOAD) return -1;
        type0_header((int)len, mf, hdr);
        pl = ax; pl_len = (int)len;
    } else {
        return -1;
    }

    il2p_scramble_block(hdr, out, IL2P_HDR_SIZE);
    il2p_rs_encode(out, IL2P_HDR_SIZE, IL2P_HDR_PARITY, out + IL2P_HDR_SIZE);
    int olen = IL2P_HDR_SIZE + IL2P_HDR_PARITY;
    if (pl_len > 0) {
        int k = encode_payload(pl, pl_len, mf, out + olen);
        if (k <= 0) return -1;
        olen += k;
    }
    return olen;
}

/* Header recibido (15 B, sin descrambling) → uhdr[13]. Devuelve símbolos
 * corregidos o -1. */
static int clarify_header(const uint8_t *rec, uint8_t *uhdr)
{
    uint8_t blk[IL2P_HDR_SIZE + IL2P_HDR_PARITY];
    memcpy(blk, rec, sizeof(blk));
    int e = il2p_rs_decode(blk, IL2P_HDR_SIZE, IL2P_HDR_PARITY);
    if (e < 0) return -1;
    il2p_descramble_block(blk, uhdr, IL2P_HDR_SIZE);
    return e;
}

/* Decodifica cabecera(uhdr) + carga codificada a trama AX.25 / bytes opacos.
 * Devuelve longitud (≥5) o -1. out ≥ FRAME_BUF_MAX. */
static int decode_frame(const uint8_t *uhdr, const uint8_t *epayload, uint8_t *out)
{
    int type = GET_TYPE(uhdr), mf = GET_FEC(uhdr), plen = GET_COUNT(uhdr);
    static uint8_t pl[IL2P_MAX_PAYLOAD];

    if (type == 1) {
        int n = type1_to_ax25(uhdr, out);
        if (n < 0) return -1;
        if (plen > 0) {
            if (decode_payload(epayload, plen, mf, pl) != plen) return -1;
            memcpy(out + n, pl, (size_t)plen);
        }
        return n + plen;
    }
    if (plen < 5) return -1;
    if (decode_payload(epayload, plen, mf, pl) != plen) return -1;
    memcpy(out, pl, (size_t)plen);
    return plen;
}

/* Para pruebas en host: trama completa sin sync → AX.25. */
int il2p_decode_frame(const uint8_t *in, uint8_t *out)
{
    uint8_t uhdr[IL2P_HDR_SIZE];
    if (clarify_header(in, uhdr) < 0) return -1;
    return decode_frame(uhdr, in + IL2P_HDR_SIZE + IL2P_HDR_PARITY, out);
}

/* ─── RX ────────────────────────────────────────────────────────────────── */

typedef enum { RX_SEARCH, RX_HEADER, RX_PAYLOAD, RX_PENDING } rx_state_t;

typedef struct {
    volatile rx_state_t state;
    uint32_t acc;
    int      bc;
    uint8_t  shdr[IL2P_HDR_SIZE + IL2P_HDR_PARITY];
    int      hc;
    uint8_t  uhdr[IL2P_HDR_SIZE];
    int      eplen, pc;
    uint8_t  spayload[IL2P_MAX_ENC];
} il2p_rx_t;

static il2p_rx_t s_rx[2];   /* 0 = modem v1, 1 = modem v2 */

static inline int popcount24(uint32_t v)
{
    int n = 0;
    while (v) { n += v & 1; v >>= 1; }
    return n;
}

static void rx_bit(il2p_rx_t *F, uint8_t bit)
{
    F->acc = ((F->acc << 1) | (bit & 1)) & 0xFFFFFF;
    switch (F->state) {
    case RX_SEARCH:
        if (popcount24(F->acc ^ IL2P_SYNC_WORD) <= 1) {
            F->state = RX_HEADER;
            F->bc = 0;
            F->hc = 0;
        }
        break;
    case RX_HEADER:
        if (++F->bc < 8) break;
        F->bc = 0;
        F->shdr[F->hc++] = (uint8_t)(F->acc & 0xFF);
        if (F->hc < IL2P_HDR_SIZE + IL2P_HDR_PARITY) break;
        if (clarify_header(F->shdr, F->uhdr) < 0) { F->state = RX_SEARCH; break; }
        {
            int type = GET_TYPE(F->uhdr), plen = GET_COUNT(F->uhdr);
            payload_props_t pp;
            int ep = payload_compute(&pp, plen, GET_FEC(F->uhdr));
            uint8_t tmp[20];
            /* Validación temprana: evita seguir con falsos sync */
            if (ep < 0 || (type == 1 && type1_to_ax25(F->uhdr, tmp) < 0) ||
                (type == 0 && plen < 5)) {
                F->state = RX_SEARCH;
                break;
            }
            F->eplen = ep;
            F->pc = 0;
            F->state = (ep == 0) ? RX_PENDING : RX_PAYLOAD;
        }
        break;
    case RX_PAYLOAD:
        if (++F->bc < 8) break;
        F->bc = 0;
        F->spayload[F->pc++] = (uint8_t)(F->acc & 0xFF);
        if (F->pc >= F->eplen) F->state = RX_PENDING;
        break;
    case RX_PENDING:
        break;   /* il2p_poll() decodifica y libera el contexto */
    }
}

/* ─── Deduplicación ─────────────────────────────────────────────────────── */

#define DEDUP_SLOTS 8
#define DEDUP_WINDOW_MS 500

static dedup_entry_t s_dedup_entries[DEDUP_SLOTS];
static dedup_t       s_dedup;

void il2p_init(bool enabled, ax25_raw_callback_t cb)
{
    s_enabled = enabled;
    s_cb      = cb;
    memset(s_rx, 0, sizeof(s_rx));
    dedup_init(&s_dedup, s_dedup_entries, DEDUP_SLOTS, (int64_t)DEDUP_WINDOW_MS * 1000);
    if (enabled)
        ESP_LOGI(TAG, "IL2P activo (max_fec=%d, tx_all=%d)", s_max_fec, s_tx_all);
}

void il2p_rx_bit(uint8_t modem_id, uint8_t nrzi_bit)
{
    if (!s_enabled || modem_id > 1) return;
    rx_bit(&s_rx[modem_id], nrzi_bit);
}

void il2p_poll(void)
{
    if (!s_enabled || !s_cb) return;
    static uint8_t frame[FRAME_BUF_MAX];
    for (int m = 0; m < 2; m++) {
        il2p_rx_t *F = &s_rx[m];
        if (F->state != RX_PENDING) continue;
        int n = decode_frame(F->uhdr, F->spayload, frame);
        F->state = RX_SEARCH;
        if (n < 5) {
            ESP_LOGD(TAG, "IL2P: carga no recuperable");
            continue;
        }
        if (!dedup_check_and_add(&s_dedup, dedup_hash(frame, (size_t)n))) {
            ESP_LOGD(TAG, "IL2P trama %d bytes (modem v%d)", n, m + 1);
            s_cb(frame, (size_t)n);
        }
    }
}

/* ─── TX ────────────────────────────────────────────────────────────────── */

static inline uint8_t bitrev8(uint8_t b)
{
    b = (uint8_t)((b & 0xF0) >> 4 | (b & 0x0F) << 4);
    b = (uint8_t)((b & 0xCC) >> 2 | (b & 0x33) << 2);
    b = (uint8_t)((b & 0xAA) >> 1 | (b & 0x55) << 1);
    return b;
}

static void il2p_tx(const uint8_t *data, size_t len, bool raw)
{
    if (!s_enabled) return;
    static uint8_t outbuf[IL2P_TX_MAX];
    int n = 0;
    for (int i = 0; i < IL2P_PREAMBLE_N; i++) outbuf[n++] = 0x55;
    outbuf[n++] = 0xF1; outbuf[n++] = 0x5E; outbuf[n++] = 0x48;
    int e = il2p_encode_frame(data, len, raw, s_max_fec, outbuf + n);
    if (e < 0) {
        ESP_LOGW(TAG, "il2p_tx: no se pudo codificar (len=%d)", (int)len);
        return;
    }
    n += e;
    /* El TX AFSK envía cada byte LSB primero; IL2P va MSB primero. */
    for (int i = 0; i < n; i++) outbuf[i] = bitrev8(outbuf[i]);

    extern Afsk *AFSK_modem;
    if (AFSK_modem) AFSK_modem->il2p_tx = true;
    AFSK_transmit((char *)outbuf, (size_t)n);
    if (AFSK_modem) AFSK_modem->il2p_tx = false;
}

void il2p_tx_frame(const uint8_t *ax25_raw, size_t len) { il2p_tx(ax25_raw, len, false); }
void il2p_tx_raw(const uint8_t *data, size_t len)       { il2p_tx(data, len, true); }

void smart_tx_frame(const uint8_t *ax25_raw, size_t len)
{
    extern void APRS_send_raw_frame(const uint8_t *buf, size_t len);
    if (!s_enabled) { APRS_send_raw_frame(ax25_raw, len); return; }

    if (s_tx_all) { il2p_tx_frame(ax25_raw, len); return; }

    /* Sólo PID 0xCC (IP) por IL2P; el resto (APRS) sigue por AX.25 */
    int off = 14;
    while (off < (int)len && !(ax25_raw[off - 1] & 1) && off + 7 <= (int)len) off += 7;
    if ((int)len >= off + 2 && ax25_raw[off + 1] == 0xCC)
        il2p_tx_frame(ax25_raw, len);
    else
        APRS_send_raw_frame(ax25_raw, len);
}
