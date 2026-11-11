/* Tests de main/rs_codec.c (Reed-Solomon GF(2^8), fcr=0) y main/il2p.c (cabecera,
 * bloques de carga, scrambler, RX por bits, routing TX). il2p.c se incluye
 * directamente porque incluye AFSK.h (-> FIFO.h), que necesita afsk_host_compat.h.
 * Dependencias de AFSK/APRS se stubean aquí. */
#include "afsk_host_compat.h"
#include "test.h"
#include "esp_timer.h"   /* test_time_set_us / test_time_advance_us */
#include "il2p.c"

/* ---- Stubs de AFSK / APRS ---- */
Afsk *AFSK_modem = NULL;
static int     tx_calls;
static int     tx_flag_during = -1;
static uint8_t tx_cap[2048];
static size_t  tx_cap_len;
void AFSK_transmit(char *buffer, size_t size)
{
    tx_calls++;
    tx_flag_during = AFSK_modem ? AFSK_modem->il2p_tx : -1;
    if (size > sizeof(tx_cap)) size = sizeof(tx_cap);
    memcpy(tx_cap, buffer, size);
    tx_cap_len = size;
}
static int aprs_calls;
void APRS_send_raw_frame(const uint8_t *buf, size_t len) { (void)buf; (void)len; aprs_calls++; }

static Afsk s_modem;

/* ---- Utilidades ---- */
static uint32_t rng_state = 0x12345678u;
static uint32_t rnd(void) { rng_state = rng_state * 1664525u + 1013904223u; return rng_state >> 8; }

/* GF(2^8) independiente (0x11D, α = 2) para verificar sin usar rs_codec.c. */
static uint8_t t_exp[512], t_log[256];
static void t_gf_init(void)
{
    int x = 1;
    for (int i = 0; i < 255; i++) {
        t_exp[i] = (uint8_t)x; t_log[x] = (uint8_t)i;
        x <<= 1; if (x & 0x100) x ^= 0x11D;
    }
    for (int i = 255; i < 512; i++) t_exp[i] = t_exp[i - 255];
}
static uint8_t t_mul(uint8_t a, uint8_t b) { return (a && b) ? t_exp[t_log[a] + t_log[b]] : 0; }

/* ¿Todos los síndromes de raíces α^0..α^(n-1) son cero? (palabra del código) */
static int syndromes_zero(const uint8_t *blk, int blen, int n)
{
    for (int i = 0; i < n; i++) {
        uint8_t a = t_exp[i], s = 0;
        for (int j = 0; j < blen; j++) s = (uint8_t)(t_mul(s, a) ^ blk[j]);
        if (s) return 0;
    }
    return 1;
}

/* Tamaño de paridad y disposición de bloques según la especificación implementada. */
typedef struct { int p, count, large_cnt, small_cnt, large, small; } layout_t;
static layout_t layout_of(int size, int max_fec)
{
    layout_t L = {0};
    L.count     = (size + (max_fec ? 238 : 246)) / (max_fec ? 239 : 247);
    L.small     = size / L.count;
    L.large     = L.small + 1;
    L.large_cnt = size - L.count * L.small;
    L.small_cnt = L.count - L.large_cnt;
    if (max_fec)                    L.p = 16;
    else if (L.small <= 61)         L.p = 2;
    else if (L.small <= 123)        L.p = 4;
    else if (L.small <= 185)        L.p = 6;
    else                            L.p = 8;
    return L;
}

/* Trama AX.25 sin FCS: direcciones (sin digipetidores), control, PID e info. */
static void put_call(uint8_t *dst, const char *call)
{
    size_t L = strlen(call);
    for (int i = 0; i < 6; i++) dst[i] = (uint8_t)((i < (int)L ? call[i] : ' ') << 1);
}
static size_t mk_frame(uint8_t *ax, const char *dst, const char *src, int cmd,
                       const uint8_t *tail, size_t tail_len)
{
    put_call(ax, dst);
    ax[6] = (uint8_t)(0x60 | (cmd ? 0x80 : 0));                  /* dest SSID 0 */
    put_call(ax + 7, src);
    ax[13] = (uint8_t)(0x61 | (cmd ? 0 : 0x80));                 /* src SSID 0, fin de dirs */
    memcpy(ax + 14, tail, tail_len);
    return 14 + tail_len;
}
static size_t mk_ui(uint8_t *ax, uint8_t pid, const uint8_t *info, size_t ilen)
{
    uint8_t tail[1100];
    tail[0] = 0x03; tail[1] = pid;
    memcpy(tail + 2, info, ilen);
    return mk_frame(ax, "APRS", "EA1ABC", 1, tail, 2 + ilen);
}

/* ================================ RS ================================ */

static const int RS_ROOTS[] = {2, 4, 6, 8, 16};

TEST(rs_encode_yields_codeword)
{
    t_gf_init();
    static const int dlens[] = {0, 1, 13, 50, 200};
    for (size_t ri = 0; ri < sizeof(RS_ROOTS) / sizeof(RS_ROOTS[0]); ri++)
        for (size_t di = 0; di < sizeof(dlens) / sizeof(dlens[0]); di++) {
            int n = RS_ROOTS[ri], dl = dlens[di];
            uint8_t blk[255], orig[255];
            for (int i = 0; i < dl; i++) blk[i] = (uint8_t)rnd();
            il2p_rs_encode(blk, dl, n, blk + dl);
            CHECK(syndromes_zero(blk, dl + n, n));
            memcpy(orig, blk, (size_t)(dl + n));
            CHECK_EQ_INT(il2p_rs_decode(blk, dl, n), 0);
            CHECK_MEM(blk, orig, (size_t)(dl + n));
        }
}

TEST(rs_corrects_up_to_t_errors)
{
    t_gf_init();
    static const int dlens[] = {13, 100};
    for (size_t ri = 0; ri < sizeof(RS_ROOTS) / sizeof(RS_ROOTS[0]); ri++)
        for (size_t di = 0; di < 2; di++) {
            int n = RS_ROOTS[ri], dl = dlens[di], blen = dl + n, t = n / 2;
            for (int trial = 0; trial < 40; trial++) {
                uint8_t cw[255], orig[255];
                for (int i = 0; i < dl; i++) cw[i] = (uint8_t)rnd();
                il2p_rs_encode(cw, dl, n, cw + dl);
                memcpy(orig, cw, (size_t)blen);
                /* t posiciones distintas (datos y paridad) con error no nulo */
                int used[255] = {0};
                for (int k = 0; k < t; k++) {
                    int p;
                    do { p = (int)(rnd() % (uint32_t)blen); } while (used[p]);
                    used[p] = 1;
                    cw[p] ^= (uint8_t)(rnd() % 255 + 1);
                }
                int ret = il2p_rs_decode(cw, dl, n);
                CHECK_EQ_INT(ret, t);
                CHECK_MEM(cw, orig, (size_t)blen);
            }
        }
}

TEST(rs_beyond_t_fails_or_miscorrects_safely)
{
    t_gf_init();
    for (size_t ri = 0; ri < sizeof(RS_ROOTS) / sizeof(RS_ROOTS[0]); ri++) {
        int n = RS_ROOTS[ri], dl = 40, blen = dl + n, t = n / 2;
        for (int trial = 0; trial < 40; trial++) {
            uint8_t cw[255], orig[255];
            for (int i = 0; i < dl; i++) cw[i] = (uint8_t)rnd();
            il2p_rs_encode(cw, dl, n, cw + dl);
            memcpy(orig, cw, (size_t)blen);
            int used[255] = {0};
            for (int k = 0; k < t + 1; k++) {               /* t+1 errores: fuera de capacidad */
                int p;
                do { p = (int)(rnd() % (uint32_t)blen); } while (used[p]);
                used[p] = 1;
                cw[p] ^= (uint8_t)(rnd() % 255 + 1);
            }
            int ret = il2p_rs_decode(cw, dl, n);
            /* Sin crash: -1, o una palabra del código distinta (miscorrección, ret<=t). */
            CHECK(ret == -1 || (ret >= 0 && ret <= t && memcmp(cw, orig, (size_t)blen) != 0));
        }
    }
}

TEST(rs_invalid_arguments)
{
    uint8_t buf[300];
    memset(buf, 0x5A, sizeof(buf));
    CHECK_EQ_INT(il2p_rs_decode(buf, 254, 2), -1);   /* blen = 256 > 255 */
    CHECK_EQ_INT(il2p_rs_decode(buf, 10, 17), -1);   /* nroots > 16 */
    CHECK_EQ_INT(il2p_rs_decode(buf, 0, 0), 0);      /* sin paridad: trivialmente válido */
}

TEST(rs_random_garbage_no_crash)
{
    t_gf_init();
    for (int trial = 0; trial < 3000; trial++) {
        int n = RS_ROOTS[rnd() % 5];
        int dl = (int)(rnd() % (uint32_t)(255 - n + 1));
        uint8_t blk[255];
        for (int i = 0; i < dl + n; i++) blk[i] = (uint8_t)rnd();
        int ret = il2p_rs_decode(blk, dl, n);
        CHECK(ret >= -1 && ret <= n / 2);
    }
}

/* ============================== IL2P ============================== */

TEST(scramble_roundtrip)
{
    for (int len = 1; len <= 80; len++) {
        uint8_t in[80], s[80], back[80];
        for (int i = 0; i < len; i++) in[i] = (uint8_t)rnd();
        il2p_scramble_block(in, s, len);
        il2p_descramble_block(s, back, len);
        CHECK_MEM(back, in, (size_t)len);
    }
    /* El scrambler no es la identidad (sanity): un bloque de ceros sale distinto. */
    uint8_t z[16] = {0}, zs[16];
    il2p_scramble_block(z, zs, 16);
    CHECK(memcmp(z, zs, 16) != 0);
}

TEST(ui_frames_roundtrip_all_pids)
{
    static const uint8_t pids[] = {0xF0, 0xCC, 0xCD, 0xCE, 0xCF, 0x01, 0x06, 0x07, 0x08};
    static uint8_t info[64];
    for (int i = 0; i < 64; i++) info[i] = (uint8_t)(i * 3 + 1);
    for (int fec = 0; fec < 2; fec++)
        for (size_t k = 0; k < sizeof(pids); k++) {
            uint8_t ax[1200], enc[1200], out[1200];
            size_t len = mk_ui(ax, pids[k], info, 20);
            int e = il2p_encode_frame(ax, len, false, fec, enc);
            CHECK(e > 0);
            int n = il2p_decode_frame(enc, out);
            CHECK_EQ_INT(n, (int)len);
            if (n == (int)len) CHECK_MEM(out, ax, len);
        }
}

TEST(ui_frames_sizes_sweep)
{
    static const int sizes[] = {0, 1, 2, 13, 61, 62, 123, 124, 185, 186, 238, 239, 240,
                                247, 248, 249, 300, 478, 479, 512, 1000, 1023};
    static uint8_t info[1023];
    for (int i = 0; i < 1023; i++) info[i] = (uint8_t)rnd();
    for (int fec = 0; fec < 2; fec++)
        for (size_t k = 0; k < sizeof(sizes) / sizeof(sizes[0]); k++) {
            uint8_t ax[1200], enc[1200], out[1200];
            size_t len = mk_ui(ax, 0xCC, info, (size_t)sizes[k]);
            int e = il2p_encode_frame(ax, len, false, fec, enc);
            CHECK(e > 0);
            int n = il2p_decode_frame(enc, out);
            CHECK_EQ_INT(n, (int)len);
            if (n == (int)len) CHECK_MEM(out, ax, len);
        }
    /* Info de 1024 B: no cabe en tipo 1 ni en tipo 0 */
    uint8_t ax[1200], enc[1200];
    size_t len = mk_ui(ax, 0xCC, info, 1023);
    CHECK_EQ_INT(il2p_encode_frame(ax, len + 1, false, 1, enc), -1);
}

TEST(supervisory_and_unnumbered_frames)
{
    /* I-frame (N(S)=3, N(R)=5, P=0) con PID CC y comando; P=1 también. */
    for (int pf = 0; pf < 2; pf++) {
        uint8_t tail[40], ax[1200], enc[1200], out[1200];
        uint8_t c = (uint8_t)((5 << 5) | (pf << 4) | (3 << 1));
        tail[0] = c; tail[1] = 0xCC; memcpy(tail + 2, "IPDATA", 6);
        size_t len = mk_frame(ax, "APRS", "EA1ABC", 1, tail, 8);
        int e = il2p_encode_frame(ax, len, false, 1, enc);
        CHECK(e > 0);
        int n = il2p_decode_frame(enc, out);
        CHECK_EQ_INT(n, (int)len);
        if (n == (int)len) CHECK_MEM(out, ax, len);
    }
    /* S-frames: RR (0x01), RNR (0x09), REJ (0x05), con y sin P, N(R)=2. Sin PID, 15 B. */
    static const uint8_t sctl[] = {0x01, 0x09, 0x05, 0x51, 0x59};
    for (size_t k = 0; k < sizeof(sctl); k++) {
        uint8_t tail[1], ax[1200], enc[1200], out[1200];
        tail[0] = sctl[k];
        size_t len = mk_frame(ax, "APRS", "EA1ABC", 1, tail, 1);
        int e = il2p_encode_frame(ax, len, false, 0, enc);
        CHECK(e > 0);
        int n = il2p_decode_frame(enc, out);
        CHECK_EQ_INT(n, (int)len);
        if (n == (int)len) CHECK_MEM(out, ax, len);
    }
    /* U-frames no UI (SABM, DISC, DM, UA, FRMR, SABM con P) en comando; UA en respuesta. */
    static const uint8_t uctl[] = {0x2F, 0x3F, 0x43, 0x0F, 0x63, 0x87};
    for (size_t k = 0; k < sizeof(uctl); k++) {
        uint8_t tail[1], ax[1200], enc[1200], out[1200];
        tail[0] = uctl[k];
        size_t len = mk_frame(ax, "APRS", "EA1ABC", 1, tail, 1);
        int e = il2p_encode_frame(ax, len, false, 1, enc);
        CHECK(e > 0);
        int n = il2p_decode_frame(enc, out);
        CHECK_EQ_INT(n, (int)len);
        if (n == (int)len) CHECK_MEM(out, ax, len);
    }
    {
        uint8_t tail[1], ax[1200], enc[1200], out[1200];
        tail[0] = 0x63;                                  /* UA como respuesta (cmd=0) */
        size_t len = mk_frame(ax, "APRS", "EA1ABC", 0, tail, 1);
        int e = il2p_encode_frame(ax, len, false, 1, enc);
        CHECK(e > 0);
        int n = il2p_decode_frame(enc, out);
        CHECK_EQ_INT(n, (int)len);
        if (n == (int)len) CHECK_MEM(out, ax, len);
    }
    /* SABME no está en la tabla IL2P: debe ir por tipo 0 y salir idéntico. */
    {
        uint8_t tail[1], ax[1200], enc[1200], out[1200];
        tail[0] = 0x6F;
        size_t len = mk_frame(ax, "APRS", "EA1ABC", 1, tail, 1);
        int e = il2p_encode_frame(ax, len, false, 1, enc);
        CHECK(e > 0);
        int n = il2p_decode_frame(enc, out);
        CHECK_EQ_INT(n, (int)len);
        if (n == (int)len) CHECK_MEM(out, ax, len);
    }
}

TEST(type0_frames_roundtrip)
{
    /* Con repetidor (ext. bit de la dirección origen a 0) → tipo 0. */
    uint8_t ax[1200], enc[1200], out[1200];
    uint8_t tail[] = {0x03, 0xF0, 'h', 'i'};
    size_t len = mk_frame(ax, "APRS", "EA1ABC", 1, tail, sizeof(tail));
    ax[13] &= (uint8_t)~1;                               /* hay más direcciones */
    uint8_t digi[7];
    put_call(digi, "WIDE1");
    digi[6] = 0x61;                                      /* ext=1, H=0 */
    memmove(ax + 14 + 7, ax + 14, len - 14);
    memcpy(ax + 14, digi, 7);
    len += 7;
    for (int fec = 0; fec < 2; fec++) {
        int e = il2p_encode_frame(ax, len, false, fec, enc);
        CHECK(e > 0);
        int n = il2p_decode_frame(enc, out);
        CHECK_EQ_INT(n, (int)len);
        if (n == (int)len) CHECK_MEM(out, ax, len);
    }
    /* Indicativo en minúsculas (no SIXBIT válido para tipo 1) → tipo 0. */
    uint8_t lc[1200];
    size_t l2 = mk_frame(lc, "aprs", "ea1abc", 1, tail, sizeof(tail));
    int e = il2p_encode_frame(lc, l2, false, 1, enc);
    CHECK(e > 0);
    int n = il2p_decode_frame(enc, out);
    CHECK_EQ_INT(n, (int)l2);
    if (n == (int)l2) CHECK_MEM(out, lc, l2);
}

TEST(raw_frames_roundtrip_all_sizes)
{
    static uint8_t data[1023];
    for (int i = 0; i < 1023; i++) data[i] = (uint8_t)rnd();
    for (int fec = 0; fec < 2; fec++)
        for (int len = 5; len <= 1023; len++) {
            uint8_t enc[1200], out[1200];
            int e = il2p_encode_frame(data, (size_t)len, true, fec, enc);
            CHECK(e > 0);
            int n = il2p_decode_frame(enc, out);
            CHECK_EQ_INT(n, len);
            if (n == len) CHECK_MEM(out, data, (size_t)len);
        }
    uint8_t enc[1200];
    CHECK_EQ_INT(il2p_encode_frame(data, 1024, true, 1, enc), -1);
    CHECK_EQ_INT(il2p_encode_frame(data, 0, true, 1, enc), -1);
}

TEST(corrects_symbol_errors_within_t)
{
    static const int raw_sizes[] = {5, 30, 61, 100, 123, 200, 239, 247, 300, 500, 1023};
    for (int fec = 0; fec < 2; fec++)
        for (size_t k = 0; k < sizeof(raw_sizes) / sizeof(raw_sizes[0]); k++) {
            int S = raw_sizes[k];
            static uint8_t data[1023], enc[1200], bad[1200], out[1200];
            for (int i = 0; i < S; i++) data[i] = (uint8_t)rnd();
            int e = il2p_encode_frame(data, (size_t)S, true, fec, enc);
            CHECK(e > 0);
            layout_t L = layout_of(S, fec);
            /* Errores en cada bloque de carga: t = p/2 símbolos distintos por bloque. */
            memcpy(bad, enc, (size_t)e);
            int off = 15;
            for (int pass = 0; pass < 2; pass++) {
                int cnt = pass == 0 ? L.large_cnt : L.small_cnt;
                int sz  = pass == 0 ? L.large : L.small;
                for (int b = 0; b < cnt; b++) {
                    int blen = sz + L.p, t = L.p / 2, used[300] = {0};
                    for (int q = 0; q < t; q++) {
                        int pos;
                        do { pos = (int)(rnd() % (uint32_t)blen); } while (used[pos]);
                        used[pos] = 1;
                        bad[off + pos] ^= (uint8_t)(rnd() % 255 + 1);
                    }
                    off += blen;
                }
            }
            CHECK_EQ_INT(off, e);
            int n = il2p_decode_frame(bad, out);
            CHECK_EQ_INT(n, S);
            if (n == S) CHECK_MEM(out, data, (size_t)S);

            /* Un símbolo corrupto en la cabecera (RS(15,13): t=1) */
            memcpy(bad, enc, (size_t)e);
            bad[rnd() % 15] ^= 0x5A;
            n = il2p_decode_frame(bad, out);
            CHECK_EQ_INT(n, S);
            if (n == S) CHECK_MEM(out, data, (size_t)S);

            /* t+1 errores en el primer bloque: fallo o salida distinta, sin crash. */
            memcpy(bad, enc, (size_t)e);
            {
                int blen = L.large_cnt > 0 ? L.large + L.p : L.small + L.p;
                int used[300] = {0};
                for (int q = 0; q <= L.p / 2 && q < blen; q++) {
                    int pos;
                    do { pos = (int)(rnd() % (uint32_t)blen); } while (used[pos]);
                    used[pos] = 1;
                    bad[15 + pos] ^= (uint8_t)(rnd() % 255 + 1);
                }
                n = il2p_decode_frame(bad, out);
                CHECK(n < 0 || n != S || memcmp(out, data, (size_t)S) != 0);
            }
        }
}

TEST(decode_garbage_no_crash)
{
    /* il2p_poll pasa siempre un buffer de IL2P_MAX_ENC: aquí se usa ese tamaño (1200 B).
     * Cabecera con RS válido y campos aleatorios (tipo, FEC, cuenta) para llegar a la
     * decodificación de carga; el resto, bytes aleatorios. */
    static uint8_t in[1200], out[1200];
    for (int trial = 0; trial < 3000; trial++) {
        for (int i = 0; i < 1200; i++) in[i] = (uint8_t)rnd();
        if (trial % 2 == 0) {
            uint8_t hdr[15];
            for (int i = 0; i < 13; i++) hdr[i] = (uint8_t)rnd();
            il2p_rs_encode(hdr, 13, 2, hdr + 13);
            memcpy(in, hdr, 15);
        }
        int n = il2p_decode_frame(in, out);
        CHECK(n >= -1 && n <= IL2P_MAX_PAYLOAD + 16);
    }
}

/* ---- RX por bits (máquina de estados) y dedup ---- */

static int    rx_n;
static size_t rx_len;
static uint8_t rx_buf[1200];
static void rx_cb(const uint8_t *b, size_t n)
{
    rx_n++;
    rx_len = n;
    if (n > sizeof(rx_buf)) n = sizeof(rx_buf);
    memcpy(rx_buf, b, n);
}
static void feed_msb(uint8_t modem, const uint8_t *b, size_t n)
{
    for (size_t i = 0; i < n; i++)
        for (int m = 0x80; m; m >>= 1) il2p_rx_bit(modem, (b[i] & m) ? 1 : 0);
}

static size_t build_rx_stream(uint8_t *stream, const uint8_t *ax, size_t len, int fec, uint8_t sync0)
{
    static uint8_t enc[1200];
    int e = il2p_encode_frame(ax, len, false, fec, enc);
    size_t n = 0;
    for (int i = 0; i < 4; i++) stream[n++] = 0x55;
    stream[n++] = sync0; stream[n++] = 0x5E; stream[n++] = 0x48;
    memcpy(stream + n, enc, (size_t)e);
    return n + (size_t)e;
}

TEST(rx_bit_state_machine_decodes)
{
    t_gf_init();
    test_time_set_us(1000000);
    static uint8_t ax[1200], stream[1400];
    size_t len = mk_ui(ax, 0xF0, (const uint8_t *)"hello il2p", 10);
    il2p_init(true, rx_cb);
    rx_n = 0;
    size_t sl = build_rx_stream(stream, ax, len, 1, 0xF1);
    feed_msb(0, stream, sl);
    il2p_poll();
    CHECK_EQ_INT(rx_n, 1);
    CHECK_EQ_INT(rx_len, len);
    if (rx_n == 1 && rx_len == len) CHECK_MEM(rx_buf, ax, len);

    /* Sync con un bit erróneo: aún se sincroniza (tolerancia de 1 bit). */
    test_time_advance_us(10000000);
    rx_n = 0;
    sl = build_rx_stream(stream, ax, len, 0, 0xF1 ^ 0x10);
    feed_msb(0, stream, sl);
    il2p_poll();
    CHECK_EQ_INT(rx_n, 1);

    /* Sync con dos bits erróneos: no se sincroniza, nada se entrega. */
    test_time_advance_us(10000000);
    rx_n = 0;
    sl = build_rx_stream(stream, ax, len, 0, 0xF1 ^ 0x11);
    feed_msb(0, stream, sl);
    il2p_poll();
    CHECK_EQ_INT(rx_n, 0);

    /* Modem 1 (v2) con la misma trama y una cabecera con un símbolo corrupto. */
    test_time_advance_us(10000000);
    rx_n = 0;
    sl = build_rx_stream(stream, ax, len, 1, 0xF1);
    stream[4 + 3 + 5] ^= 0x33;                          /* cabecera: byte 5 */
    feed_msb(1, stream, sl);
    il2p_poll();
    CHECK_EQ_INT(rx_n, 1);
    il2p_init(false, NULL);
}

TEST(rx_duplicates_suppressed_by_dedup_window)
{
    t_gf_init();
    test_time_set_us(5000000);
    static uint8_t ax[1200], stream[1400];
    size_t len = mk_ui(ax, 0xCC, (const uint8_t *)"dup", 3);
    il2p_init(true, rx_cb);
    rx_n = 0;
    size_t sl = build_rx_stream(stream, ax, len, 1, 0xF1);
    feed_msb(0, stream, sl);
    il2p_poll();
    CHECK_EQ_INT(rx_n, 1);
    test_time_advance_us(100000);                       /* 100 ms: dentro de la ventana de 500 ms */
    feed_msb(1, stream, sl);                            /* el otro modem la ve también */
    il2p_poll();
    CHECK_EQ_INT(rx_n, 1);
    test_time_advance_us(600000);                       /* fuera de ventana */
    feed_msb(0, stream, sl);
    il2p_poll();
    CHECK_EQ_INT(rx_n, 2);
    il2p_init(false, NULL);
}

TEST(smart_tx_routing_and_il2p_tx)
{
    static uint8_t ax[1200];
    static const uint8_t info[] = "routing";
    AFSK_modem = &s_modem;
    memset(&s_modem, 0, sizeof(s_modem));

    /* IL2P desactivado: todo por AX.25 */
    il2p_init(false, NULL);
    aprs_calls = 0; tx_calls = 0;
    size_t len = mk_ui(ax, 0xCC, info, sizeof(info) - 1);
    smart_tx_frame(ax, len);
    CHECK_EQ_INT(aprs_calls, 1);
    CHECK_EQ_INT(tx_calls, 0);

    /* IL2P activo, tx_all=false: PID CC por IL2P, resto por AX.25 */
    il2p_init(true, NULL);
    il2p_set_options(true, false);
    aprs_calls = 0; tx_calls = 0;
    smart_tx_frame(ax, len);                            /* PID CC */
    CHECK_EQ_INT(tx_calls, 1);
    CHECK_EQ_INT(aprs_calls, 0);
    CHECK_EQ_INT(tx_flag_during, 1);                    /* il2p_tx activo durante AFSK_transmit */
    CHECK_EQ_INT(s_modem.il2p_tx, 0);                   /* y restaurado después */

    /* Cabecera del stream transmitido (bits invertidos por AFSK): 4x 0x55, F1 5E 48. */
    uint8_t wire[2048];
    for (size_t i = 0; i < tx_cap_len; i++) wire[i] = bitrev8(tx_cap[i]);
    CHECK_EQ_INT(wire[0], 0x55); CHECK_EQ_INT(wire[3], 0x55);
    CHECK_EQ_INT(wire[4], 0xF1); CHECK_EQ_INT(wire[5], 0x5E); CHECK_EQ_INT(wire[6], 0x48);
    uint8_t out[1200];
    int n = il2p_decode_frame(wire + 7, out);
    CHECK_EQ_INT(n, (int)len);
    if (n == (int)len) CHECK_MEM(out, ax, len);

    /* PID F0 (APRS): AX.25 */
    len = mk_ui(ax, 0xF0, info, sizeof(info) - 1);
    aprs_calls = 0; tx_calls = 0;
    smart_tx_frame(ax, len);
    CHECK_EQ_INT(aprs_calls, 1);
    CHECK_EQ_INT(tx_calls, 0);

    /* tx_all=true: también PID F0 por IL2P */
    il2p_set_options(true, true);
    aprs_calls = 0; tx_calls = 0;
    smart_tx_frame(ax, len);
    CHECK_EQ_INT(tx_calls, 1);
    CHECK_EQ_INT(aprs_calls, 0);

    /* TUN opaco (00 00 08 00 + IPv4 de 20 B) por il2p_tx_raw: tipo 0 */
    uint8_t tun[24] = {0x00, 0x00, 0x08, 0x00, 0x45, 0x00, 0x00, 0x14};
    for (int i = 8; i < 24; i++) tun[i] = (uint8_t)(i * 7);
    tx_calls = 0;
    il2p_tx_raw(tun, sizeof(tun));
    CHECK_EQ_INT(tx_calls, 1);
    for (size_t i = 0; i < tx_cap_len; i++) wire[i] = bitrev8(tx_cap[i]);
    n = il2p_decode_frame(wire + 7, out);
    CHECK_EQ_INT(n, (int)sizeof(tun));
    if (n == (int)sizeof(tun)) CHECK_MEM(out, tun, sizeof(tun));

    /* IL2P desactivado: il2p_tx_* no transmiten nada */
    il2p_init(false, NULL);
    tx_calls = 0;
    il2p_tx_frame(ax, len);
    il2p_tx_raw(tun, sizeof(tun));
    CHECK_EQ_INT(tx_calls, 0);
    AFSK_modem = NULL;
}

TEST(garbage_tx_input_does_not_crash)
{
    /* Tramas no codificables (demasiado grandes, vacías) no deben transmitirse ni romper. */
    static uint8_t big[1100];
    memset(big, 0x42, sizeof(big));
    AFSK_modem = &s_modem;
    il2p_init(true, NULL);
    il2p_set_options(false, false);
    tx_calls = 0;
    il2p_tx_raw(big, 1100);                             /* > 1023: no codificable */
    il2p_tx_frame(big, 0);
    CHECK_EQ_INT(tx_calls, 0);
    il2p_init(false, NULL);
    AFSK_modem = NULL;
}

int main(void)
{
    RUN(rs_encode_yields_codeword);
    RUN(rs_corrects_up_to_t_errors);
    RUN(rs_beyond_t_fails_or_miscorrects_safely);
    RUN(rs_invalid_arguments);
    RUN(rs_random_garbage_no_crash);
    RUN(scramble_roundtrip);
    RUN(ui_frames_roundtrip_all_pids);
    RUN(ui_frames_sizes_sweep);
    RUN(supervisory_and_unnumbered_frames);
    RUN(type0_frames_roundtrip);
    RUN(raw_frames_roundtrip_all_sizes);
    RUN(corrects_symbol_errors_within_t);
    RUN(decode_garbage_no_crash);
    RUN(rx_bit_state_machine_decodes);
    RUN(rx_duplicates_suppressed_by_dedup_window);
    RUN(smart_tx_routing_and_il2p_tx);
    RUN(garbage_tx_input_does_not_crash);
    return test_summary();
}
