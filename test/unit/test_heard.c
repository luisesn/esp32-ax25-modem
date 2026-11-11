// Tests de main/heard.c: tabla de estaciones oídas, posiciones (sin comprimir, comprimida,
// Mic-E, con marca de tiempo), SSID, reemplazo, reloj falso, salidas con buffer pequeño.
#include "test.h"
#include "heard.h"
#include "cJSON.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

void test_time_set_us(int64_t us);
void test_time_advance_us(int64_t us);

// ---- Construcción de tramas AX.25 (sin CRC) ------------------------------
static void put_addr(uint8_t *p, const char *call, int ssid, int last)
{
    int i = 0;
    for (; i < 6 && call[i]; i++) p[i] = (uint8_t)((uint8_t)call[i] << 1);
    for (; i < 6; i++) p[i] = (uint8_t)(' ' << 1);
    p[6] = (uint8_t)(0x60 | ((ssid & 0x0F) << 1) | (last ? 1 : 0));
}

// Trama: destino APRS, origen call-ssid, sin repetidores, UI, PID 0xF0, info dada (len il).
static size_t build(uint8_t *f, const char *src, int ssid, const uint8_t *info, size_t il)
{
    put_addr(f, "APRS", 0, 0);
    put_addr(f + 7, src, ssid, 1);
    f[14] = 0x03;
    f[15] = 0xF0;
    memcpy(f + 16, info, il);
    return 16 + il;
}

static size_t build_s(uint8_t *f, const char *src, int ssid, const char *info)
{
    return build(f, src, ssid, (const uint8_t *)info, strlen(info));
}

// Mic-E: destino de 6 caracteres ya codificados (sin desplazar), 'dest'.
static size_t build_mice(uint8_t *f, const char *dest, const char *info, size_t il)
{
    for (int i = 0; i < 6; i++) f[i] = (uint8_t)((uint8_t)dest[i] << 1);
    f[6] = 0x60;
    put_addr(f + 7, "EA3MIC", 0, 1);
    f[14] = 0x03;
    f[15] = 0xF0;
    memcpy(f + 16, info, il);
    return 16 + il;
}

#define BUF 1024
static char jbuf[BUF];
static char tbuf[BUF];
static uint8_t frame[BUF];

static cJSON *get_json(void)
{
    int n = heard_json(jbuf, sizeof(jbuf));
    if (n < 0) return NULL;
    return cJSON_Parse(jbuf);
}

static cJSON *find_entry(cJSON *root, const char *call)
{
    cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "heard");
    cJSON *e;
    cJSON_ArrayForEach(e, arr) {
        cJSON *c = cJSON_GetObjectItemCaseSensitive(e, "call");
        if (cJSON_IsString(c) && strcmp(c->valuestring, call) == 0) return e;
    }
    return NULL;
}

static int count_entries(cJSON *root)
{
    cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "heard");
    return cJSON_GetArraySize(arr);
}

// Reloj: 1000 s al inicio de cada test (heard_init no lo toca).
static void fresh(void)
{
    heard_init();
    test_time_set_us(1000LL * 1000000LL);
}

// ---- Tests ----------------------------------------------------------------

TEST(uncompressed_position_with_ssid)
{
    fresh();
    size_t n = build_s(frame, "EA1ABC", 7, "!4903.50N/07201.75W-PHG5132");
    heard_note(frame, n, 3);
    cJSON *r = get_json();
    CHECK(r != NULL);
    if (!r) return;
    cJSON *e = find_entry(r, "EA1ABC-7");
    CHECK(e != NULL);
    if (e) {
        CHECK(fabs(cJSON_GetObjectItem(e, "lat")->valuedouble - 49.058333) < 0.0005);
        CHECK(fabs(cJSON_GetObjectItem(e, "lon")->valuedouble + 72.029167) < 0.0005);
        CHECK_EQ_INT(cJSON_GetObjectItem(e, "n")->valueint, 1);
        CHECK(fabs(cJSON_GetObjectItem(e, "q")->valuedouble - 3.0) < 0.05);
    }
    cJSON_Delete(r);
}

TEST(sign_of_coordinates)
{
    fresh();
    // Sur y este: lat negativa y lon positiva.
    size_t n = build_s(frame, "EA1SE", 0, "!3352.00S/15112.00E-");
    heard_note(frame, n, 0);
    cJSON *r = get_json();
    cJSON *e = r ? find_entry(r, "EA1SE") : NULL;
    CHECK(e != NULL);
    if (e) {
        CHECK(fabs(cJSON_GetObjectItem(e, "lat")->valuedouble + 33.866667) < 0.0005);
        CHECK(fabs(cJSON_GetObjectItem(e, "lon")->valuedouble - 151.200000) < 0.0005);
        CHECK(cJSON_GetObjectItem(e, "q") == NULL);   // q <= 0: sin dato
    }
    cJSON_Delete(r);
}

TEST(timestamped_position)
{
    fresh();
    size_t n = build_s(frame, "EA4TS", 0, "/092345z4903.50N/07201.75W>");
    heard_note(frame, n, 2);
    cJSON *r = get_json();
    cJSON *e = r ? find_entry(r, "EA4TS") : NULL;
    CHECK(e != NULL);
    if (e) CHECK(fabs(cJSON_GetObjectItem(e, "lat")->valuedouble - 49.058333) < 0.0005);
    cJSON_Delete(r);
}

TEST(compressed_position)
{
    // Ejemplo de APRS101 cap. 9: "!/5L!!<*e7>7P[" = 49°30'N 72°45'W aprox.
    fresh();
    size_t n = build_s(frame, "EA2XYZ", 0, "!/5L!!<*e7>7P[");
    heard_note(frame, n, 0);
    cJSON *r = get_json();
    cJSON *e = r ? find_entry(r, "EA2XYZ") : NULL;
    CHECK(e != NULL);
    if (e) {
        CHECK(fabs(cJSON_GetObjectItem(e, "lat")->valuedouble - 49.5) < 0.001);
        CHECK(fabs(cJSON_GetObjectItem(e, "lon")->valuedouble + 72.75) < 0.001);
    }
    cJSON_Delete(r);
}

TEST(compressed_position_with_timestamp)
{
    fresh();
    // '/' con marca de tiempo y posición comprimida (13 bytes tras los 7 de la hora).
    size_t n = build_s(frame, "EA2CMP", 0, "/092345z/5L!!<*e7>7P[");
    heard_note(frame, n, 0);
    cJSON *r = get_json();
    cJSON *e = r ? find_entry(r, "EA2CMP") : NULL;
    CHECK(e != NULL);
    if (e) CHECK(fabs(cJSON_GetObjectItem(e, "lat")->valuedouble - 49.5) < 0.001);
    cJSON_Delete(r);
}

TEST(mice_position_known_vector)
{
    // Vector calculado a mano (APRS101 cap. 10):
    //  destino "490SUP": lat 49 03.50 N (4,9,0,3,5,0; N = 'S' en la 4.ª; offset +100 = 'U' en la 5.ª;
    //                    W = 'P' en la 6.ª).
    //  info: '`' + lon (byte 28+12 = '(' -> 12 + 100 = 112; minutos 25 = '5'; centésimas 75 = 'g')
    //        + 3 bytes rumbo/velocidad + símbolo '>' + tabla '/'.
    //  Esperado: lat = 49 + 3.50/60 = 49.058333; lon = -(112 + 25.75/60) = -112.429167.
    fresh();
    const char info[] = "`(5gabc>/";
    size_t n = build_mice(frame, "490SUP", info, 9);
    heard_note(frame, n, 0);
    cJSON *r = get_json();
    cJSON *e = r ? find_entry(r, "EA3MIC") : NULL;
    CHECK(e != NULL);
    if (e) {
        double lat = cJSON_GetObjectItem(e, "lat")->valuedouble;
        double lon = cJSON_GetObjectItem(e, "lon")->valuedouble;
        CHECK(fabs(lat - 49.058333) < 0.0005);
        CHECK(fabs(lon + 112.429167) < 0.0005);   // BUG si da -12.429 (falta el offset de 100)
    }
    cJSON_Delete(r);
}

TEST(mice_real_captured_frame)
{
    // Trama Mic-E real (captura 2026-11-11, test/demod/corpus/real_20261111_b.wav):
    // EA1IUX-9>TS2QVW,ED1YCF-3*,WIDE1*,EA1HOK-3*:`z6dl ...
    // Cálculo a mano: destino TS2QVW -> 43 21.67 N; lon: 'z'(122-28=94)+100=194 -> 4 grados,
    // min '6' = 26, cent 'd' = 72 -> 4 26.72 W => lat 43.36117, lon -4.44533.
    static const uint8_t raw[] = { 0xa8, 0xa6, 0x64, 0xa2, 0xac, 0xae, 0x60, 0x8a, 0x82, 0x62, 0x92, 0xaa, 0xb0, 0xf2, 0x8a, 0x88, 0x62, 0xb2, 0x86, 0x8c, 0xe6, 0xae, 0x92, 0x88, 0x8a, 0x62, 0x40, 0xe0, 0x8a, 0x82, 0x62, 0x90, 0x9e, 0x96, 0xe7, 0x03, 0xf0, 0x60, 0x7a, 0x36, 0x64, 0x6c, 0x20, 0x1c, 0x3e, 0x2f, 0x60, 0x22, 0x34, 0x76, 0x7d, 0x5f, 0x25, 0x0d };
    fresh();
    heard_note(raw, sizeof raw, 0);
    cJSON *r = get_json();
    cJSON *e = r ? find_entry(r, "EA1IUX-9") : NULL;
    CHECK(e != NULL);
    if (e) {
        CHECK(fabs(cJSON_GetObjectItem(e, "lat")->valuedouble - 43.36117) < 0.0005);
        CHECK(fabs(cJSON_GetObjectItem(e, "lon") ->valuedouble + 4.44533) < 0.0005);
    }
    cJSON_Delete(r);
}

TEST(track_history_dedup_and_age_filter)
{
    heard_init();
    test_time_set_us(1000LL * 1000000LL);
    size_t n;
    n = build_s(frame, "EA1TRK", 1, "!4321.67N/00426.72W>");     heard_note(frame, n, 0);
    test_time_advance_us(60LL * 1000000LL);
    n = build_s(frame, "EA1TRK", 1, "!4321.67N/00426.72W>");     heard_note(frame, n, 0);  // misma posición: no añade
    test_time_advance_us(60LL * 1000000LL);
    n = build_s(frame, "EA1TRK", 1, "!4322.67N/00426.72W>");     heard_note(frame, n, 0);  // ~1,8 km al norte
    n = build_s(frame, "EA1OTH", 0, "!4300.00N/00500.00W>");     heard_note(frame, n, 0);
    n = build_s(frame, "EA1TXT", 0, "hola sin posicion");        heard_note(frame, n, 0);  // sin posición: no entra
    heard_pt_t pts[HEARD_TRACK_MAX];
    int c = heard_track(pts, HEARD_TRACK_MAX, 0);
    CHECK_EQ_INT(c, 3);
    CHECK_STR(pts[0].call, "EA1TRK-1");
    CHECK_EQ_INT((int)pts[0].age, 120);          // más antiguo primero
    CHECK(fabsf(pts[1].lat - pts[0].lat) > 0.01f);
    CHECK_STR(pts[2].call, "EA1OTH");
    // filtro por edad: sólo lo de los últimos 30 s
    CHECK_EQ_INT(heard_track(pts, HEARD_TRACK_MAX, 30), 2);
    // anillo: 200 posiciones distintas -> se queda en HEARD_TRACK_MAX
    for (int i = 0; i < 200; i++) {
        char info[40]; snprintf(info, sizeof info, "!43%02d.00N/004%02d.00W>", i % 60, (i / 60) % 60 + 1);
        n = build_s(frame, "EA1RNG", 0, info); heard_note(frame, n, 0);
        test_time_advance_us(1000000LL);
    }
    CHECK_EQ_INT(heard_track(pts, HEARD_TRACK_MAX, 0), HEARD_TRACK_MAX);   // anillo lleno, sin desbordar
    heard_init();
    CHECK_EQ_INT(heard_track(pts, HEARD_TRACK_MAX, 0), 0);
}

TEST(mice_position_without_offset)
{
    // Igual pero sin offset (dígito 5.ª = '5', no 'U'): 12°25.75'W.
    fresh();
    const char info[] = "`(5gabc>/";
    size_t n = build_mice(frame, "490S5P", info, 9);
    heard_note(frame, n, 0);
    cJSON *r = get_json();
    cJSON *e = r ? find_entry(r, "EA3MIC") : NULL;
    CHECK(e != NULL);
    if (e) {
        CHECK(fabs(cJSON_GetObjectItem(e, "lat")->valuedouble - 49.058333) < 0.0005);
        CHECK(fabs(cJSON_GetObjectItem(e, "lon")->valuedouble + 12.429167) < 0.0005);
    }
    cJSON_Delete(r);
}

TEST(mice_too_short_no_position)
{
    fresh();
    const char info[] = "`(5g";           // < 9 bytes de info: sin posición
    size_t n = build_mice(frame, "490SUP", info, 4);
    heard_note(frame, n, 0);
    cJSON *r = get_json();
    cJSON *e = r ? find_entry(r, "EA3MIC") : NULL;
    CHECK(e != NULL);
    if (e) CHECK(cJSON_GetObjectItem(e, "lat") == NULL);
    cJSON_Delete(r);
}

TEST(third_party_position_goes_to_inner_station)
{
    fresh();
    size_t n = build_s(frame, "ED2ZAD", 3, "}ED2ZAD>APBM1S,TCPIP,ED2ZAD-3*:@112236z4322.21N/00245.80WrPHG0000RD985");
    heard_note(frame, n, 2);
    cJSON *r = get_json();
    CHECK(r != NULL);
    if (!r) return;
    cJSON *e = find_entry(r, "ED2ZAD");
    CHECK(e != NULL);
    if (e) {
        CHECK(fabs(cJSON_GetObjectItem(e, "lat")->valuedouble - 43.37017) < 0.0005);
        CHECK(fabs(cJSON_GetObjectItem(e, "lon")->valuedouble + 2.76333) < 0.0005);
    }
    CHECK(find_entry(r, "ED2ZAD-3") != NULL);   // el gateway también se oyó
    cJSON_Delete(r);
}

TEST(status_has_no_position_and_q_average)
{
    fresh();
    size_t n = build_s(frame, "EA5Q", 0, ">Estacion de prueba");
    heard_note(frame, n, 3);
    heard_note(frame, n, 4);
    heard_note(frame, n, 0);    // sin dato de calidad: no entra en la media
    heard_note(frame, n, -2);
    cJSON *r = get_json();
    cJSON *e = r ? find_entry(r, "EA5Q") : NULL;
    CHECK(e != NULL);
    if (e) {
        CHECK_EQ_INT(cJSON_GetObjectItem(e, "n")->valueint, 4);
        CHECK(fabs(cJSON_GetObjectItem(e, "q")->valuedouble - 3.5) < 0.05);
        CHECK(cJSON_GetObjectItem(e, "lat") == NULL);
        CHECK(cJSON_GetObjectItem(e, "lon") == NULL);
    }
    cJSON_Delete(r);
}

TEST(ssid_distinguishes_stations)
{
    fresh();
    size_t n = build_s(frame, "EA6S", 0, ">a");
    heard_note(frame, n, 0);
    n = build_s(frame, "EA6S", 1, ">b");
    heard_note(frame, n, 0);
    n = build_s(frame, "EA6S", 15, ">c");   // SSID 15 -> "-15" cabe en el nombre
    heard_note(frame, n, 0);
    cJSON *r = get_json();
    CHECK_EQ_INT(count_entries(r), 3);
    CHECK(find_entry(r, "EA6S") != NULL);
    CHECK(find_entry(r, "EA6S-1") != NULL);
    CHECK(find_entry(r, "EA6S-15") != NULL);
    cJSON_Delete(r);
}

TEST(heard_seen_recent_with_fake_clock)
{
    fresh();                                          // t = 1000 s
    test_time_set_us(10LL * 1000000LL);
    size_t n = build_s(frame, "EA7R", 2, ">x");       // oída en t=10 s
    heard_note(frame, n, 0);
    test_time_set_us(20LL * 1000000LL);
    CHECK(heard_seen_recent("EA7R", 2, 30));          // hace 10 s
    CHECK(!heard_seen_recent("EA7R", 0, 30));         // otro SSID
    CHECK(!heard_seen_recent("EA7Q", 2, 30));         // no oída
    test_time_set_us(40LL * 1000000LL);
    CHECK(heard_seen_recent("EA7R", 2, 30));          // exactamente 30 s: dentro
    test_time_set_us(41LL * 1000000LL);
    CHECK(!heard_seen_recent("EA7R", 2, 30));         // 31 s: fuera
}

TEST(replaces_oldest_when_full)
{
    fresh();
    char call[16];
    for (int i = 1; i <= 21; i++) {
        snprintf(call, sizeof(call), "C%05d", i);      // 6 caracteres
        test_time_set_us((1000LL + i) * 1000000LL);
        size_t n = build_s(frame, call, 0, ">x");
        heard_note(frame, n, 0);
    }
    cJSON *r = get_json();
    CHECK(r != NULL);
    if (r) {
        CHECK_EQ_INT(count_entries(r), 20);
        CHECK(find_entry(r, "C00001") == NULL);    // la más antigua se reemplazó
        CHECK(find_entry(r, "C00002") != NULL);
        CHECK(find_entry(r, "C00021") != NULL);
    }
    cJSON_Delete(r);
    test_time_set_us(1100LL * 1000000LL);
    CHECK(!heard_seen_recent("C00001", 0, 1000));
    CHECK(heard_seen_recent("C00021", 0, 1000));
}

TEST(json_order_and_ages)
{
    fresh();
    test_time_set_us(100LL * 1000000LL);
    size_t n = build_s(frame, "EA8A", 0, ">a");
    heard_note(frame, n, 0);
    test_time_set_us(105LL * 1000000LL);
    n = build_s(frame, "EA8B", 0, ">b");
    heard_note(frame, n, 0);
    test_time_set_us(110LL * 1000000LL);
    int len = heard_json(jbuf, sizeof(jbuf));
    CHECK(len > 0);
    CHECK_EQ_INT(len, (int)strlen(jbuf));
    cJSON *r = cJSON_Parse(jbuf);
    CHECK(r != NULL);
    if (r) {
        cJSON *arr = cJSON_GetObjectItem(r, "heard");
        CHECK_EQ_INT(cJSON_GetArraySize(arr), 2);
        // Más reciente primero.
        CHECK_STR(cJSON_GetObjectItem(cJSON_GetArrayItem(arr, 0), "call")->valuestring, "EA8B");
        CHECK_EQ_INT(cJSON_GetObjectItem(cJSON_GetArrayItem(arr, 0), "age")->valueint, 5);
        CHECK_EQ_INT(cJSON_GetObjectItem(cJSON_GetArrayItem(arr, 1), "age")->valueint, 10);
    }
    cJSON_Delete(r);
}

TEST(json_buffer_sizes)
{
    fresh();
    // Vacía: "{\"heard\":[]}" = 12 caracteres. heard_json exige n >= 16 (mínimo fijo, aunque
    // quepa menos); con 16 bytes devuelve la longitud sin contar el NUL.
    char small[32];
    CHECK_EQ_INT(heard_json(small, 0), -1);
    CHECK_EQ_INT(heard_json(small, 15), -1);
    CHECK_EQ_INT(heard_json(NULL, 100), -1);
    CHECK_EQ_INT(heard_json(small, 13), -1);
    CHECK_EQ_INT(heard_json(small, 16), 12);
    CHECK_STR(small, "{\"heard\":[]}");

    // Con estaciones: cualquier tamaño insuficiente devuelve -1 sin desbordar.
    size_t n = build_s(frame, "EA9A", 0, "!4903.50N/07201.75W-");
    heard_note(frame, n, 4);
    n = build_s(frame, "EA9B", 1, ">x");
    heard_note(frame, n, 2);
    int full = heard_json(jbuf, sizeof(jbuf));
    CHECK(full > 0);
    for (int sz = 0; sz <= full + 1; sz++) {
        char *b = malloc((size_t)sz > 0 ? (size_t)sz : 1);
        int r = heard_json(b, (size_t)sz);
        if (r >= 0) {
            CHECK_EQ_INT(r, full);
            CHECK(sz > full);
        } else {
            CHECK(sz <= full);
        }
        free(b);
    }
}

TEST(list_console_and_buffers)
{
    fresh();
    CHECK_EQ_INT(heard_list(tbuf, sizeof(tbuf)), 19);
    CHECK_STR(tbuf, "No stations heard\r\n");

    size_t n = build_s(frame, "EB1CON", 0, "!4903.50N/07201.75W-");
    heard_note(frame, n, 3);
    n = build_s(frame, "EB1CO2", 4, ">x");
    heard_note(frame, n, 0);
    int len = heard_list(tbuf, sizeof(tbuf));
    CHECK(len > 0);
    CHECK_EQ_INT(len, (int)strlen(tbuf));
    CHECK(strstr(tbuf, "EB1CO2-4") != NULL);
    CHECK(strstr(tbuf, "EB1CON") != NULL);
    CHECK(strstr(tbuf, " P\r\n") != NULL);            // la estación con posición lleva "P"

    // Buffers pequeños: nunca desbordan y siempre terminan en NUL.
    for (size_t sz = 1; sz < (size_t)len + 2; sz++) {
        char *b = malloc(sz);
        memset(b, 'Z', sz);
        int r = heard_list(b, sz);
        CHECK(r >= 0 && (size_t)r < sz + 1);
        CHECK(memchr(b, '\0', sz) != NULL);
        free(b);
    }
    CHECK_EQ_INT(heard_list(tbuf, 0), 0);
    CHECK_EQ_INT(heard_list(NULL, 10), 0);
}

TEST(init_clears_table)
{
    fresh();
    size_t n = build_s(frame, "EC1X", 0, ">x");
    heard_note(frame, n, 1);
    CHECK(heard_seen_recent("EC1X", 0, 100));
    heard_init();
    CHECK(!heard_seen_recent("EC1X", 0, 100));
    CHECK_EQ_INT(heard_json(jbuf, sizeof(jbuf)) > 0, 1);
    CHECK_STR(jbuf, "{\"heard\":[]}");
}

TEST(malformed_frames_no_crash)
{
    fresh();
    heard_note(NULL, 0, 0);
    heard_note(NULL, 100, 0);
    heard_note(frame, 0, 0);
    heard_note(frame, 15, 0);
    // Origen con todos los bytes de dirección con bit final a 0 y sin control/PID.
    memset(frame, 0x40, sizeof(frame));
    heard_note(frame, 16, 2);
    heard_note(frame, 100, 2);
    // Posiciones truncadas: sólo el indicador.
    size_t n = build_s(frame, "EDT1", 0, "!");
    heard_note(frame, n, 0);
    n = build_s(frame, "EDT2", 0, "!4903.5");
    heard_note(frame, n, 0);
    n = build_s(frame, "EDT3", 0, "/0923");
    heard_note(frame, n, 0);
    n = build_s(frame, "EDT4", 0, "!9999.99N/99999.99W-");    // fuera de rango
    heard_note(frame, n, 0);
    n = build_s(frame, "EDT5", 0, "`");
    heard_note(frame, n, 0);
    cJSON *r = get_json();
    CHECK(r != NULL);
    cJSON *e = r ? find_entry(r, "EDT4") : NULL;
    CHECK(e != NULL);
    if (e) CHECK(cJSON_GetObjectItem(e, "lat") == NULL);
    cJSON_Delete(r);
}

TEST(random_garbage_no_crash)
{
    fresh();
    uint32_t seed = 987654u;
    for (int it = 0; it < 3000; it++) {
        seed = seed * 1664525u + 1013904223u;
        size_t len = (seed >> 8) % 330;
        for (size_t i = 0; i < len; i++) {
            seed = seed * 1664525u + 1013904223u;
            frame[i] = (uint8_t)(seed >> 24);
        }
        if (len > 16 && (it % 3) == 0) {   // cabecera válida con info aleatoria (incluye '!', '`', '/')
            static const char starts[] = "!=/@'`";
            put_addr(frame, "APRS", 0, 0);
            put_addr(frame + 7, "EGR1", 0, 1);
            frame[14] = 0x03; frame[15] = 0xF0;
            frame[16] = (uint8_t)starts[it % 6];
        }
        heard_note(frame, len, (int)(seed % 6) - 1);
    }
    int r = heard_json(jbuf, sizeof(jbuf));
    CHECK(r >= 0);
    CHECK(heard_list(tbuf, sizeof(tbuf)) >= 0);
}

int main(void)
{
    RUN(uncompressed_position_with_ssid);
    RUN(third_party_position_goes_to_inner_station);
    RUN(sign_of_coordinates);
    RUN(timestamped_position);
    RUN(compressed_position);
    RUN(compressed_position_with_timestamp);
    RUN(mice_position_known_vector);
    RUN(mice_real_captured_frame);
    RUN(mice_position_without_offset);
    RUN(mice_too_short_no_position);
    RUN(status_has_no_position_and_q_average);
    RUN(ssid_distinguishes_stations);
    RUN(heard_seen_recent_with_fake_clock);
    RUN(replaces_oldest_when_full);
    RUN(json_order_and_ages);
    RUN(json_buffer_sizes);
    RUN(list_console_and_buffers);
    RUN(track_history_dedup_and_age_filter);
    RUN(init_clears_table);
    RUN(malformed_frames_no_crash);
    RUN(random_garbage_no_crash);
    return test_summary();
}
