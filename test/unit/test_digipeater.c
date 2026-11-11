// Tests de main/digipeater.c (WIDEn-N, alias, indicativo propio, dedup, bucles, basura).
#include "test.h"
#include "digipeater.h"
#include "cJSON.h"
#include "LibAPRS-esp32-i2s/src/AX25.h"
#include <stdlib.h>
#include <string.h>

// Stub de afsk_queue_tx_frame (stubs_digipeater.c)
extern uint8_t stub_tx_buf[];
extern size_t  stub_tx_len;
extern int     stub_tx_count;
void stub_tx_reset(void);
void test_time_set_us(int64_t us);
void test_time_advance_us(int64_t us);

// ---- Construcción de tramas AX.25 ----------------------------------------
// Dirección: 6 caracteres desplazados <<1 + byte SSID (0x60 | ssid<<1 | H<<7 | last).
static void put_addr(uint8_t *p, const char *call, int ssid, int last, int h)
{
    int i = 0;
    for (; i < 6 && call[i]; i++) p[i] = (uint8_t)((uint8_t)call[i] << 1);
    for (; i < 6; i++) p[i] = (uint8_t)(' ' << 1);
    p[6] = (uint8_t)(0x60 | ((ssid & 0x0F) << 1) | (h ? 0x80 : 0) | (last ? 1 : 0));
}

// path: lista "CALL[-N][*],..." ('*' = bit H). Devuelve longitud de la trama.
static size_t build_frame(uint8_t *f, const char *dst, const char *src, int sssid,
                          const char *path, const char *info)
{
    put_addr(f, dst, 0, 0, 0);
    put_addr(f + 7, src, sssid, path[0] == '\0', 0);
    size_t o = 14;
    const char *p = path;
    while (*p) {
        char call[7] = {0};
        int n = 0, ssid = 0, h = 0;
        while (*p && *p != ',' && *p != '-' && *p != '*') { if (n < 6) call[n++] = *p; p++; }
        if (*p == '-') { p++; while (*p >= '0' && *p <= '9') { ssid = ssid * 10 + (*p - '0'); p++; } }
        if (*p == '*') { h = 1; p++; }
        int more = (*p == ',');
        if (more) p++;
        put_addr(f + o, call, ssid, !more, h);
        o += 7;
    }
    f[o++] = 0x03;   // UI
    f[o++] = 0xF0;   // sin capa 3
    size_t il = strlen(info);
    memcpy(f + o, info, il);
    return o + il;
}

// Configura el digipeador: indicativo propio MYCALL-9 (aprs.*), digi RPT1-0, alias WIDE1-1, WIDE2-2, RELAY.
static void digi_setup(bool enabled)
{
    const char *json =
        "{\"aprs\":{\"callsign\":\"MYCALL\",\"ssid\":9},"
        " \"digi\":{\"enabled\":false,\"alias\":[\"WIDE1-1\",\"WIDE2-2\",\"RELAY\"],"
        "          \"callsign\":\"RPT1\",\"ssid\":0}}";
    cJSON *cfg = cJSON_Parse(json);
    digi_init(cfg);
    cJSON_Delete(cfg);
    digi_set_enabled(enabled);
}

// Avanza el reloj 100 s para que el dedup de tests anteriores caduque.
static void fresh(void)
{
    static int64_t t = 1000000000LL;
    t += 100000000LL;
    test_time_set_us(t);
    stub_tx_reset();
}

#define BUF 400
static uint8_t frame[BUF], expect[BUF];

// Procesa frame[0..len) y comprueba si se emitió la trama `exp` (si exp==NULL, no se emitió nada).
static void check_digi(const uint8_t *in, size_t len, const char *exp_dst, const char *exp_src,
                       int exp_sssid, const char *exp_path, const char *info)
{
    CHECK(digi_process_frame(in, len));
    CHECK_EQ_INT(stub_tx_count, 1);
    size_t el = build_frame(expect, exp_dst, exp_src, exp_sssid, exp_path, info);
    CHECK_EQ_INT(stub_tx_len, el);
    CHECK_MEM(stub_tx_buf, expect, el < stub_tx_len ? el : stub_tx_len);
}

static void check_no_digi(const uint8_t *in, size_t len)
{
    CHECK(!digi_process_frame(in, len));
    CHECK_EQ_INT(stub_tx_count, 0);
}

// ---- Tests ----------------------------------------------------------------

TEST(disabled_no_digipeats)
{
    fresh();
    digi_setup(false);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", "!4903.50N/07201.75W-");
    check_no_digi(frame, len);
    CHECK(!digi_is_enabled());
    digi_set_enabled(true);
    CHECK(digi_is_enabled());
    digi_set_enabled(false);
}

TEST(wide1_1_becomes_rpt_and_wide1_used)
{
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", "wide1");
    check_digi(frame, len, "APRS", "EA1AAA", 0, "RPT1*,WIDE1*", "wide1");
    CHECK_EQ_INT(stub_tx_len, len + 7);   // se inserta una dirección
}

TEST(wide2_2_decrements_ssid)
{
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE2-2", "wide2-2");
    check_digi(frame, len, "APRS", "EA1AAA", 0, "RPT1*,WIDE2-1", "wide2-2");
}

TEST(wide2_1_second_hop)
{
    // Segundo salto WIDEn-N: un digi configurado con WIDE2-2 debería repetir WIDE2-1
    // (WIDE2-1 -> WIDE2*). El alias casa sólo por SSID exacto, así que no lo repite.
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE2-1", "wide2-1");
    // BUG: con alias WIDE2-2 la trama WIDE2-1 se ignora (alias exacto); esperado RPT1*,WIDE2*
    bool ok = digi_process_frame(frame, len);
    size_t el = build_frame(expect, "APRS", "EA1AAA", 0, "RPT1*,WIDE2*", "wide2-1");
    KNOWN_BUG(ok && stub_tx_len == el, "digipeater.c: alias por SSID exacto");
}


TEST(configured_alias_relay)
{
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "RELAY", "relay");
    check_digi(frame, len, "APRS", "EA1AAA", 0, "RPT1*,RELAY*", "relay");
}

TEST(unknown_aliases_rejected)
{
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE3-3", "no-wide3");
    check_no_digi(frame, len);
    fresh();
    len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-2", "no-wide1-2");   // alias exacto WIDE1-1
    check_no_digi(frame, len);
    fresh();
    len = build_frame(frame, "APRS", "EA1AAA", 0, "NOTHERE", "no-alias");
    check_no_digi(frame, len);
}

TEST(second_hop_after_used_digi_is_handled)
{
    fresh();
    digi_setup(true);
    // EA2XYZ ya repitió (H=1): el siguiente salto pendiente es WIDE1-1.
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "EA2XYZ*,WIDE1-1", "chain");
    check_digi(frame, len, "APRS", "EA1AAA", 0, "EA2XYZ*,RPT1*,WIDE1*", "chain");
}

TEST(fully_used_path_not_digipeated)
{
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1*", "used");
    check_no_digi(frame, len);
    fresh();
    len = build_frame(frame, "APRS", "EA1AAA", 0, "EA2XYZ*,WIDE1*", "used2");
    check_no_digi(frame, len);
}

TEST(explicit_own_call_sets_h_without_inserting)
{
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "RPT1-0", "explicit");
    CHECK(digi_process_frame(frame, len));
    CHECK_EQ_INT(stub_tx_count, 1);
    CHECK_EQ_INT(stub_tx_len, len);                  // no se inserta nada
    size_t el = build_frame(expect, "APRS", "EA1AAA", 0, "RPT1*", "explicit");
    CHECK_MEM(stub_tx_buf, expect, el);
}

TEST(own_origin_not_digipeated)
{
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "RPT1", 0, "WIDE1-1", "own-digi");
    check_no_digi(frame, len);
    fresh();
    len = build_frame(frame, "APRS", "MYCALL", 9, "WIDE1-1", "own-aprs");
    check_no_digi(frame, len);
    fresh();
    // Mismo indicativo base con otro SSID: sí se repite.
    len = build_frame(frame, "APRS", "MYCALL", 3, "WIDE1-1", "other-ssid");
    CHECK(digi_process_frame(frame, len));
    CHECK_EQ_INT(stub_tx_count, 1);
}

TEST(loop_own_call_already_used_is_dropped)
{
    fresh();
    digi_setup(true);
    // Nuestra trama ya pasó por nosotros (RPT1 con H=1) y vuelve con WIDE1-1 pendiente.
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "RPT1*,WIDE1-1", "loop");
    check_no_digi(frame, len);
    fresh();
    len = build_frame(frame, "APRS", "EA1AAA", 0, "RPT1*,WIDE2-2", "loop2");
    check_no_digi(frame, len);
}

TEST(duplicate_suppressed_within_ttl_then_allowed)
{
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", "dup-test");
    CHECK(digi_process_frame(frame, len));
    CHECK_EQ_INT(stub_tx_count, 1);
    stub_tx_reset();
    test_time_advance_us(10000000LL);                // +10 s, dentro del TTL de 30 s
    CHECK(!digi_process_frame(frame, len));
    CHECK_EQ_INT(stub_tx_count, 0);
    test_time_advance_us(31000000LL);                // +41 s, fuera del TTL
    CHECK(digi_process_frame(frame, len));
    CHECK_EQ_INT(stub_tx_count, 1);
}

TEST(dedup_distinguishes_payload)
{
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", "payload-A");
    CHECK(digi_process_frame(frame, len));
    len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", "payload-B");
    CHECK(digi_process_frame(frame, len));
    CHECK_EQ_INT(stub_tx_count, 2);          // payload distinto: no se suprime
}

TEST(dedup_ignores_path_state)
{
    // La misma información con WIDE1-1 y con WIDE1-1 de otro origen: el hash
    // excluye el path, así que la segunda se suprime aunque el SSID del salto varíe.
    fresh();
    digi_setup(true);
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", "same-info");
    CHECK(digi_process_frame(frame, len));
    stub_tx_reset();
    len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE2-2", "same-info");
    CHECK(!digi_process_frame(frame, len));
    CHECK_EQ_INT(stub_tx_count, 0);
}

TEST(max_frame_length_respected)
{
    fresh();
    digi_setup(true);
    // Trama justo en el límite: +7 bytes de la nueva dirección no caben.
    size_t info = AX25_MAX_FRAME_LEN - 23;           // 14 + 7 + 2 de cabecera con WIDE1-1
    char *big = malloc(info + 1);
    memset(big, 'A', info); big[info] = '\0';
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", big);
    CHECK(len > AX25_MAX_FRAME_LEN - 8);
    if (len > AX25_MAX_FRAME_LEN - 7) {
        check_no_digi(frame, len > BUF ? BUF : len);
    }
    free(big);
    // Justo cabiendo: sí se repite y la salida mide exactamente AX25_MAX_FRAME_LEN.
    fresh();
    info = AX25_MAX_FRAME_LEN - 30;
    big = malloc(info + 1);
    memset(big, 'B', info); big[info] = '\0';
    len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", big);
    CHECK(digi_process_frame(frame, len));
    CHECK_EQ_INT(stub_tx_len, AX25_MAX_FRAME_LEN);
    free(big);
}

TEST(truncated_frames_no_crash)
{
    fresh();
    digi_setup(true);
    size_t full = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", "trunc");
    for (size_t l = 0; l <= full; l++) {
        stub_tx_reset();
        uint8_t copy[BUF];
        memcpy(copy, frame, full);
        bool r = digi_process_frame(copy, l);
        if (l < 16) CHECK(!r);
        (void)r;
    }
    // Sin el byte de PID: no hay info ni PID, no se debe repetir.
    stub_tx_reset();
    CHECK(!digi_process_frame(frame, 14 + 7 + 1));
    CHECK_EQ_INT(stub_tx_count, 0);
}

TEST(path_without_end_bit_rejected)
{
    fresh();
    digi_setup(true);
    // Dirección origen sin bit final y 9 repetidores sin bit final: malformada.
    uint8_t f[BUF];
    memset(f, 0x40, sizeof(f));
    for (int i = 0; i < 6; i++) f[i] = (uint8_t)('A' << 1);
    put_addr(f, "APRS", 0, 0, 0);
    put_addr(f + 7, "EA1AAA", 0, 0, 0);
    check_no_digi(f, 200);
}

TEST(random_garbage_no_crash)
{
    fresh();
    digi_setup(true);
    uint32_t seed = 12345u;
    uint8_t g[BUF];
    for (int it = 0; it < 3000; it++) {
        seed = seed * 1664525u + 1013904223u;
        size_t len = (seed >> 8) % (BUF + 1);
        for (size_t i = 0; i < len; i++) {
            seed = seed * 1664525u + 1013904223u;
            g[i] = (uint8_t)(seed >> 24);
        }
        // Cadenas de direcciones con bits finales aleatorios y origen de nuestro indicativo.
        if (len > 14 && (it & 1)) { put_addr(g, "APRS", 0, 0, 0); put_addr(g + 7, "EA1AAA", 0, 0, 0); }
        (void)digi_process_frame(g, len);
    }
    CHECK(1);
}

TEST(digi_init_parsing_and_fallback)
{
    // Alias como cadena, indicativo propio tomado de aprs.* si falta digi.callsign.
    cJSON *cfg = cJSON_Parse("{\"aprs\":{\"callsign\":\"XX1YYY\",\"ssid\":3},"
                             "\"digi\":{\"enabled\":true,\"alias\":\"WIDE1-1\"}}");
    digi_init(cfg);
    cJSON_Delete(cfg);
    CHECK(digi_is_enabled());
    fresh();
    size_t len = build_frame(frame, "APRS", "EA1AAA", 0, "WIDE1-1", "fallback");
    CHECK(digi_process_frame(frame, len));
    size_t el = build_frame(expect, "APRS", "EA1AAA", 0, "XX1YYY-3*,WIDE1*", "fallback");
    CHECK_MEM(stub_tx_buf, expect, el);
    // Sin sección digi: digi_init no cambia el estado.
    cfg = cJSON_Parse("{\"aprs\":{\"callsign\":\"XX1YYY\"}}");
    digi_init(cfg);
    cJSON_Delete(cfg);
    CHECK(digi_is_enabled());
    digi_init(NULL);   // no debe fallar
    CHECK(digi_is_enabled());
    digi_set_enabled(false);
}

TEST(process_null_and_short_inputs)
{
    fresh();
    digi_setup(true);
    CHECK(!digi_process_frame(NULL, 0));
    CHECK(!digi_process_frame(frame, 0));
    CHECK(!digi_process_frame(frame, 15));
    CHECK_EQ_INT(stub_tx_count, 0);
}

int main(void)
{
    RUN(disabled_no_digipeats);
    RUN(wide1_1_becomes_rpt_and_wide1_used);
    RUN(wide2_2_decrements_ssid);
    RUN(wide2_1_second_hop);
    RUN(configured_alias_relay);
    RUN(unknown_aliases_rejected);
    RUN(second_hop_after_used_digi_is_handled);
    RUN(fully_used_path_not_digipeated);
    RUN(explicit_own_call_sets_h_without_inserting);
    RUN(own_origin_not_digipeated);
    RUN(loop_own_call_already_used_is_dropped);
    RUN(duplicate_suppressed_within_ttl_then_allowed);
    RUN(dedup_distinguishes_payload);
    RUN(dedup_ignores_path_state);
    RUN(max_frame_length_respected);
    RUN(truncated_frames_no_crash);
    RUN(path_without_end_bit_rejected);
    RUN(random_garbage_no_crash);
    RUN(digi_init_parsing_and_fallback);
    RUN(process_null_and_short_inputs);
    return test_summary();
}
