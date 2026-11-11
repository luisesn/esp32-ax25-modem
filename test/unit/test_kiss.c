/* Tests de main/kiss.c (framing KISS, comandos P/SlotTime, contextos, encode).
 * kiss.c se incluye directamente porque incluye AFSK.h (-> FIFO.h), que necesita
 * los macros de afsk_host_compat.h antes de cualquier otro include. */
#include "afsk_host_compat.h"
#include "test.h"
#include "kiss.c"

#define FEND  KISS_FEND
#define FESC  KISS_FESC
#define TFEND KISS_TFEND
#define TFESC KISS_TFESC
#define MAXF  AX25_MAX_FRAME_LEN

/* ---- Stubs de dependencias (transport.c y AFSK.cpp no se enlazan) ---- */
static uint8_t txbuf[2 * MAXF + 8];
static size_t  txlen;
static int     tx_calls;
void transport_write(const uint8_t *buf, size_t len)
{
    if (len > sizeof(txbuf)) len = sizeof(txbuf);
    memcpy(txbuf, buf, len);
    txlen = len;
    tx_calls++;
}

static uint8_t g_persist = 63;
static uint32_t g_slot = 100;
static int csma_cfg_calls;
static uint8_t csma_last_persist;
static uint32_t csma_last_slot;
void afsk_csma_get(uint8_t *persist, uint32_t *slot_ms) { *persist = g_persist; *slot_ms = g_slot; }
void afsk_csma_configure(uint8_t persist, uint32_t slot_ms)
{
    g_persist = persist; g_slot = slot_ms;
    csma_cfg_calls++;
    csma_last_persist = persist; csma_last_slot = slot_ms;
}

/* ---- Registro de tramas entregadas por el callback ---- */
#define MAX_REC 16
static uint8_t rec[MAX_REC][2 * MAXF];
static size_t  rec_len[MAX_REC];
static int     rec_n;
static void on_frame(const uint8_t *buf, size_t len)
{
    if (rec_n >= MAX_REC) { rec_n++; return; }
    if (len > sizeof(rec[0])) len = sizeof(rec[0]);
    memcpy(rec[rec_n], buf, len);
    rec_len[rec_n] = len;
    rec_n++;
}
static void reset_rec(void) { rec_n = 0; memset(rec_len, 0, sizeof(rec_len)); }

static void feed(kiss_ctx_t *c, const uint8_t *b, size_t n)
{
    for (size_t i = 0; i < n; i++) kiss_ctx_rx_byte(c, b[i]);
}

/* ---- Tests ---- */

TEST(escapes_decode)
{
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *c = kiss_ctx_new();
    CHECK(c != NULL);
    const uint8_t in[] = {FEND, 0x00, FESC, TFEND, FESC, TFESC, FEND};
    feed(c, in, sizeof(in));
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], 2);
    const uint8_t exp[] = {FEND, FESC};
    CHECK_MEM(rec[0], exp, 2);
    kiss_ctx_free(c);
}

TEST(escapes_encode)
{
    const uint8_t payload[] = {FEND, FESC, 0x41};
    kiss_send_frame(payload, sizeof(payload));
    const uint8_t exp[] = {FEND, 0x00, FESC, TFEND, FESC, TFESC, 0x41, FEND};
    CHECK_EQ_INT(tx_calls > 0, 1);
    CHECK_EQ_INT(txlen, sizeof(exp));
    CHECK_MEM(txbuf, exp, sizeof(exp));
}

TEST(roundtrip_all_bytes)
{
    /* Los 256 valores de byte (incluidos FEND y FESC) y una trama de tamaño máximo. */
    static uint8_t p[256];
    for (int i = 0; i < 256; i++) p[i] = (uint8_t)i;
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *c = kiss_ctx_new();
    kiss_send_frame(p, sizeof(p));
    feed(c, txbuf, txlen);
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], 256);
    CHECK_MEM(rec[0], p, 256);

    static uint8_t big[MAXF];
    for (int i = 0; i < MAXF; i++) big[i] = (i & 1) ? FEND : FESC;
    reset_rec();
    kiss_send_frame(big, MAXF);      /* 2*MAXF+3 B codificados: cabe en el buffer de encode */
    feed(c, txbuf, txlen);
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], MAXF);
    CHECK_MEM(rec[0], big, MAXF);
    kiss_ctx_free(c);
}

TEST(fragmented_byte_by_byte)
{
    const uint8_t payload[] = {0x01, FEND, 0x02, FESC, 0x03, 0x04};
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *c = kiss_ctx_new();
    kiss_send_frame(payload, sizeof(payload));
    uint8_t wire[64];
    size_t wn = txlen;
    memcpy(wire, txbuf, wn);
    for (size_t i = 0; i < wn; i++) {
        kiss_ctx_rx_byte(c, wire[i]);
        if (i + 1 < wn) CHECK_EQ_INT(rec_n, 0);   /* nada hasta el FEND final */
    }
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], sizeof(payload));
    CHECK_MEM(rec[0], payload, sizeof(payload));
    kiss_ctx_free(c);
}

TEST(empty_and_fend_only)
{
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *c = kiss_ctx_new();
    const uint8_t a[] = {FEND, FEND, FEND, FEND, FEND};   /* sólo FEND */
    feed(c, a, sizeof(a));
    const uint8_t b[] = {FEND, 0x00, FEND};               /* cmd DATA sin payload */
    feed(c, b, sizeof(b));
    const uint8_t d[] = {FEND, 0x00, FEND, FEND};         /* y FEND doble */
    feed(c, d, sizeof(d));
    CHECK_EQ_INT(rec_n, 0);
    /* Tras las tramas vacías, una válida se entrega completa. */
    const uint8_t e[] = {0x00, 0x01, FEND};
    feed(c, e, sizeof(e));
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], 1);
    CHECK_EQ_INT(rec[0][0], 0x01);
    /* Bytes antes del primer FEND se ignoran. */
    reset_rec();
    kiss_ctx_reset(c);
    const uint8_t f[] = {0x55, 0x66, FEND, 0x00, 0x7A, FEND};
    feed(c, f, sizeof(f));
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec[0][0], 0x7A);
    kiss_ctx_free(c);
}

TEST(overflow_discarded_then_recovers)
{
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *c = kiss_ctx_new();
    /* MAXF+1 bytes: se descarta entera (no se entrega truncada). */
    static uint8_t big[MAXF + 1];
    memset(big, 0x55, sizeof(big));
    uint8_t hdr[] = {FEND, 0x00};
    uint8_t end[] = {FEND};
    feed(c, hdr, sizeof(hdr));
    feed(c, big, sizeof(big));
    feed(c, end, sizeof(end));
    CHECK_EQ_INT(rec_n, 0);

    /* Trama mucho mayor (con escapes) también descartada, sin desbordar el buffer. */
    static uint8_t huge[3 * MAXF];
    for (size_t i = 0; i < sizeof(huge); i++) huge[i] = (i & 1) ? FEND : 0x33;
    feed(c, hdr, sizeof(hdr));
    for (size_t i = 0; i < sizeof(huge); i++) {
        uint8_t b = huge[i];
        if (b == FEND) { uint8_t esc[] = {FESC, TFEND}; feed(c, esc, 2); }
        else kiss_ctx_rx_byte(c, b);
    }
    feed(c, end, sizeof(end));
    CHECK_EQ_INT(rec_n, 0);

    /* Trama válida siguiente: entregada. */
    const uint8_t ok[] = {FEND, 0x00, 0x10, 0x20, 0x30, FEND};
    feed(c, ok, sizeof(ok));
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], 3);
    CHECK_EQ_INT(rec[0][2], 0x30);

    /* Exactamente MAXF bytes: se entrega. */
    reset_rec();
    static uint8_t exact[MAXF];
    memset(exact, 0x77, sizeof(exact));
    feed(c, hdr, sizeof(hdr));
    feed(c, exact, sizeof(exact));
    feed(c, end, sizeof(end));
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], MAXF);
    kiss_ctx_free(c);
}

TEST(pslot_commands)
{
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *c = kiss_ctx_new();
    csma_cfg_calls = 0;
    g_persist = 63; g_slot = 100;

    /* P = 0x3F */
    const uint8_t p[] = {FEND, 0x02, 0x3F, FEND};
    feed(c, p, sizeof(p));
    CHECK_EQ_INT(csma_cfg_calls, 1);
    CHECK_EQ_INT(csma_last_persist, 0x3F);
    CHECK_EQ_INT(csma_last_slot, 100);

    /* P con valor escapado (0xC0 = FEND -> FESC TFEND) */
    const uint8_t pe[] = {FEND, 0x02, FESC, TFEND, FEND};
    feed(c, pe, sizeof(pe));
    CHECK_EQ_INT(csma_cfg_calls, 2);
    CHECK_EQ_INT(csma_last_persist, 0xC0);

    /* SlotTime = 10 -> 100 ms (unidades de 10 ms) */
    const uint8_t s[] = {FEND, 0x03, 0x0A, FEND};
    feed(c, s, sizeof(s));
    CHECK_EQ_INT(csma_cfg_calls, 3);
    CHECK_EQ_INT(csma_last_slot, 100);
    CHECK_EQ_INT(csma_last_persist, 0xC0);

    /* SlotTime = 25 -> 250 ms */
    const uint8_t s2[] = {FEND, 0x03, 25, FEND};
    feed(c, s2, sizeof(s2));
    CHECK_EQ_INT(csma_last_slot, 250);

    /* TXDELAY (0x01), FullDuplex (0x05) y SetHardware (0x06) se ignoran. */
    const uint8_t ign[] = {FEND, 0x01, 30, FEND, FEND, 0x05, 1, FEND, FEND, 0x06, 1, FEND};
    feed(c, ign, sizeof(ign));
    CHECK_EQ_INT(csma_cfg_calls, 4);            /* sólo el SlotTime de 25 cuenta */

    /* P sin payload (len 0): no configura. */
    const uint8_t e[] = {FEND, 0x02, FEND};
    feed(c, e, sizeof(e));
    CHECK_EQ_INT(csma_cfg_calls, 4);

    /* Comandos de configuración no se entregan como tramas. */
    CHECK_EQ_INT(rec_n, 0);
    kiss_ctx_free(c);
}

TEST(contexts_independent)
{
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *a = kiss_ctx_new(), *b = kiss_ctx_new();
    CHECK(a != NULL && b != NULL && a != b);
    /* Trama de A a medias, B completa en medio, luego A termina. */
    const uint8_t a1[] = {FEND, 0x00, 0x41, 0x42};
    const uint8_t b1[] = {FEND, 0x00, 0x58, 0x59, FEND};
    const uint8_t a2[] = {0x43, FEND};
    feed(a, a1, sizeof(a1));
    feed(b, b1, sizeof(b1));
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], 2);
    CHECK_MEM(rec[0], "XY", 2);
    feed(a, a2, sizeof(a2));
    CHECK_EQ_INT(rec_n, 2);
    CHECK_EQ_INT(rec_len[1], 3);
    CHECK_MEM(rec[1], "ABC", 3);

    /* Un reset de A no afecta a B. */
    reset_rec();
    const uint8_t a3[] = {FEND, 0x00, 0x11};
    const uint8_t b2[] = {0x22, FEND};
    feed(a, a3, sizeof(a3));
    const uint8_t b3[] = {FEND, 0x00, 0x33};
    feed(b, b3, sizeof(b3));
    kiss_ctx_reset(a);
    feed(a, (const uint8_t[]){0x44, FEND}, 2);      /* sin FEND inicial: descartado */
    feed(b, b2, sizeof(b2));
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], 2);
    CHECK_EQ_INT(rec[0][0], 0x33);
    CHECK_EQ_INT(rec[0][1], 0x22);
    kiss_ctx_free(a);
    kiss_ctx_free(b);
}

TEST(reset_mid_frame)
{
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *c = kiss_ctx_new();
    const uint8_t p1[] = {FEND, 0x00, 0x41, 0x42};
    feed(c, p1, sizeof(p1));
    kiss_ctx_reset(c);
    const uint8_t p2[] = {0x43, FEND};              /* resto de la trama cortada */
    feed(c, p2, sizeof(p2));
    CHECK_EQ_INT(rec_n, 0);
    const uint8_t p3[] = {FEND, 0x00, 0x44, FEND};
    feed(c, p3, sizeof(p3));
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec[0][0], 0x44);
    kiss_ctx_free(c);
}

TEST(default_context_api)
{
    reset_rec();
    kiss_init(on_frame);
    const uint8_t p[] = {FEND, 0x00, 0x61, 0x62, FEND};
    for (size_t i = 0; i < sizeof(p); i++) kiss_rx_byte(p[i]);
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], 2);
    /* kiss_reset a mitad de trama */
    const uint8_t q[] = {FEND, 0x00, 0x63};
    for (size_t i = 0; i < sizeof(q); i++) kiss_rx_byte(q[i]);
    kiss_reset();
    const uint8_t r[] = {0x64, FEND, FEND, 0x00, 0x65, FEND};
    for (size_t i = 0; i < sizeof(r); i++) kiss_rx_byte(r[i]);
    CHECK_EQ_INT(rec_n, 2);
    CHECK_EQ_INT(rec_len[1], 1);
    CHECK_EQ_INT(rec[1][0], 0x65);
}

TEST(invalid_escape_discards_byte)
{
    /* FESC seguido de byte no válido: el byte inválido se descarta, la trama sigue. */
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *c = kiss_ctx_new();
    const uint8_t p[] = {FEND, 0x00, 0x41, FESC, 0x58, 0x42, FEND};
    feed(c, p, sizeof(p));
    CHECK_EQ_INT(rec_n, 1);
    CHECK_EQ_INT(rec_len[0], 2);
    CHECK_MEM(rec[0], "AB", 2);
    kiss_ctx_free(c);
}

TEST(fend_after_fesc_terminates_frame)
{
    /* Un FEND tras FESC es un error de protocolo, pero un FEND siempre debe cerrar la
     * trama. El decodificador lo consume como si fuera el byte escapado y la trama
     * "AA" se fusiona con la siguiente. */
    reset_rec();
    kiss_init(on_frame);
    kiss_ctx_t *c = kiss_ctx_new();
    const uint8_t p[] = {FEND, 0x00, 0xAA, FESC, FEND, 0x00, 0xBB, FEND};
    feed(c, p, sizeof(p));
    // BUG: FEND tras FESC se traga y las tramas {AA} y {BB} se fusionan en una sola
    // trama {AA,00,BB}. Esperado: rec_n==2 con {AA} y {BB}.
    KNOWN_BUG(rec_n == 2 && rec_len[0] == 1, "kiss.c: FEND tras FESC");
    kiss_ctx_free(c);
}

int main(void)
{
    RUN(escapes_decode);
    RUN(escapes_encode);
    RUN(roundtrip_all_bytes);
    RUN(fragmented_byte_by_byte);
    RUN(empty_and_fend_only);
    RUN(overflow_discarded_then_recovers);
    RUN(pslot_commands);
    RUN(contexts_independent);
    RUN(reset_mid_frame);
    RUN(default_context_api);
    RUN(invalid_escape_discards_byte);
    RUN(fend_after_fesc_terminates_frame);
    return test_summary();
}
