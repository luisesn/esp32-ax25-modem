// Tests de main/config_validate.c: plantilla config.json.defaults válida, rangos, tipos,
// formatos (indicativo, path, SSID), claves desconocidas y buffer de error pequeño.
#include "test.h"
#include "config_validate.h"
#include "cJSON.h"
#include <stdlib.h>
#include <string.h>

// Ruta a la plantilla: TEST_ROOT si está definido; si no, se deriva de __FILE__
// (test/unit/test_config_validate.c -> raíz del repo). Así no hace falta tocar CMake.
static char *read_defaults(void)
{
    char path[1024];
    const char *root = getenv("TEST_ROOT");
    if (root && *root) {
        snprintf(path, sizeof(path), "%s/main/spiffs_data/config.json.defaults", root);
    } else {
        snprintf(path, sizeof(path), "%s", __FILE__);
        for (int up = 0; up < 3; up++) {          // quita nombre, "unit" y "test"
            char *slash = strrchr(path, '/');
            if (!slash) break;
            *slash = '\0';
        }
        strncat(path, "/main/spiffs_data/config.json.defaults", sizeof(path) - strlen(path) - 1);
    }
    FILE *f = fopen(path, "rb");
    if (!f) { printf("   no se pudo abrir %s\n", path); return NULL; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = malloc((size_t)n + 1);
    if (buf) { size_t r = fread(buf, 1, (size_t)n, f); buf[r] = '\0'; }
    fclose(f);
    return buf;
}

#define ERRBUF 256

// Valida un JSON. Devuelve el resultado y deja el mensaje en err.
static bool validate_json(const char *json, char *err, size_t errlen)
{
    cJSON *cfg = cJSON_Parse(json);
    if (!cfg) { printf("   JSON de prueba no válido: %s\n", json); return false; }
    bool ok = config_validate(cfg, err, errlen);
    cJSON_Delete(cfg);
    return ok;
}

static void expect_ok(const char *json)
{
    char err[ERRBUF] = "sentinela";
    bool ok = validate_json(json, err, sizeof(err));
    if (!ok) printf("   debería validar: %s -> %s\n", json, err);
    CHECK(ok);
    CHECK_STR(err, "");                          // en éxito el buffer queda vacío
}

// Debe fallar; el mensaje no vacío, <=120 caracteres y (si se da) contener `substr`.
static void expect_err(const char *json, const char *substr)
{
    char err[ERRBUF] = "";
    bool ok = validate_json(json, err, sizeof(err));
    if (ok) printf("   debería fallar: %s\n", json);
    CHECK(!ok);
    CHECK(strlen(err) > 0);
    CHECK(strlen(err) <= 120);
    if (substr) {
        if (!strstr(err, substr)) printf("   mensaje \"%s\" no contiene \"%s\" (%s)\n", err, substr, json);
        CHECK(strstr(err, substr) != NULL);
    }
}

// ---- Tests ----------------------------------------------------------------

TEST(defaults_template_is_valid)
{
    char *txt = read_defaults();
    CHECK(txt != NULL);
    if (!txt) return;
    cJSON *cfg = cJSON_Parse(txt);
    CHECK(cfg != NULL);
    if (cfg) {
        char err[ERRBUF] = "x";
        bool ok = config_validate(cfg, err, sizeof(err));
        if (!ok) printf("   plantilla rechazada: %s\n", err);
        CHECK(ok);
        CHECK_STR(err, "");
        // Mutaciones: un valor fuera de rango en la plantilla debe rechazarse.
        cJSON *aprs = cJSON_GetObjectItemCaseSensitive(cfg, "aprs");
        cJSON *ssid = cJSON_GetObjectItemCaseSensitive(aprs, "ssid");
        cJSON_SetNumberValue(ssid, 16);
        CHECK(!config_validate(cfg, err, sizeof(err)));
        CHECK(strstr(err, "aprs.ssid") != NULL);
        cJSON_Delete(cfg);
    }
    free(txt);
}

TEST(root_must_be_object)
{
    char err[ERRBUF];
    expect_err("[]", "objeto");
    expect_err("5", "objeto");
    expect_err("\"x\"", "objeto");
    expect_err("null", "objeto");
    expect_err("true", "objeto");
    // Llamada directa con NULL (no hay JSON que parsear).
    CHECK(!config_validate(NULL, err, sizeof(err)));
    CHECK(strlen(err) > 0);
}

TEST(empty_and_unknown_keys_accepted)
{
    expect_ok("{}");
    expect_ok("{\"foo\":1,\"bar\":[1,2],\"baz\":{\"x\":null}}");
    expect_ok("{\"aprs\":{\"unknown_key\":true,\"otra\":\"x\"}}");
    expect_ok("{\"Aprs\":\"no es sección\"}");       // sensible a mayúsculas: clave desconocida
    expect_ok("{\"version\":\"1.11.0\"}");
    expect_err("{\"version\":5}", "version");
}

TEST(section_must_be_object)
{
    expect_err("{\"aprs\":\"x\"}", "aprs debe ser un objeto");
    expect_err("{\"ap\":[]}", "ap debe ser un objeto");
    expect_err("{\"gps\":5}", "gps debe ser un objeto");
    expect_err("{\"tx\":null}", "tx debe ser un objeto");
    expect_err("{\"wifi\":{}}", "wifi debe ser un array");
    expect_err("{\"wifi\":\"x\"}", "wifi debe ser un array");
}

TEST(aprs_callsign_and_ssid)
{
    expect_ok("{\"aprs\":{\"callsign\":\"N0CALL\"}}");
    expect_ok("{\"aprs\":{\"callsign\":\"A\"}}");
    expect_ok("{\"aprs\":{\"callsign\":\"ABC123\"}}");
    expect_err("{\"aprs\":{\"callsign\":5}}", "aprs.callsign debe ser una cadena");
    expect_err("{\"aprs\":{\"callsign\":\"n0call\"}}", "aprs.callsign");
    expect_err("{\"aprs\":{\"callsign\":\"TOOLONG7\"}}", "aprs.callsign");
    expect_err("{\"aprs\":{\"callsign\":\"\"}}", "aprs.callsign");
    expect_err("{\"aprs\":{\"callsign\":\"N0-CAL\"}}", "aprs.callsign");

    expect_ok("{\"aprs\":{\"ssid\":0}}");
    expect_ok("{\"aprs\":{\"ssid\":15}}");
    expect_err("{\"aprs\":{\"ssid\":-1}}", "aprs.ssid debe estar entre 0 y 15");
    expect_err("{\"aprs\":{\"ssid\":16}}", "aprs.ssid debe estar entre 0 y 15");
    expect_err("{\"aprs\":{\"ssid\":\"3\"}}", "aprs.ssid debe ser un número");
}

TEST(aprs_delay_range)
{
    expect_ok("{\"aprs\":{\"post_rx_tx_delay_ms\":0}}");
    expect_ok("{\"aprs\":{\"post_rx_tx_delay_ms\":5000}}");
    expect_err("{\"aprs\":{\"post_rx_tx_delay_ms\":-1}}", "post_rx_tx_delay_ms debe estar entre 0 y 5000");
    expect_err("{\"aprs\":{\"post_rx_tx_delay_ms\":5001}}", "post_rx_tx_delay_ms");
    expect_err("{\"aprs\":{\"post_rx_tx_delay_ms\":\"1050\"}}", "post_rx_tx_delay_ms debe ser un número");
}

TEST(aprs_path_formats)
{
    expect_ok("{\"aprs\":{\"path\":\"\"}}");
    expect_ok("{\"aprs\":{\"path\":\"WIDE1-1,WIDE2-2\"}}");
    expect_ok("{\"aprs\":{\"path\":\"RELAY\"}}");
    expect_ok("{\"aprs\":{\"path\":\" WIDE1-1 , WIDE2-2 \"}}");   // espacios tolerados
    expect_ok("{\"aprs\":{\"path\":\"RPT1-15\"}}");
    expect_ok("{\"aprs\":{\"ack_path\":\"WIDE1-1\"}}");
    expect_err("{\"aprs\":{\"path\":\"A,B,C\"}}", "aprs.path admite como máximo 2");
    expect_err("{\"aprs\":{\"path\":\"WIDE1-16\"}}", "aprs.path");
    expect_err("{\"aprs\":{\"path\":\"WIDE1-\"}}", "aprs.path");
    expect_err("{\"aprs\":{\"path\":\"WIDE1-123\"}}", "aprs.path");
    expect_err("{\"aprs\":{\"path\":\"WIDE1-x\"}}", "aprs.path");
    expect_err("{\"aprs\":{\"path\":\"A,,B\"}}", "repetidor vacío");
    expect_err("{\"aprs\":{\"path\":\"wide1-1\"}}", "aprs.path");
    expect_err("{\"aprs\":{\"path\":\"ABCDEFG-1\"}}", "aprs.path");
    expect_err("{\"aprs\":{\"path\":5}}", "aprs.path debe ser una cadena");
    expect_err("{\"aprs\":{\"ack_path\":\"WIDE1-1,WIDE2-2,RELAY\"}}", "aprs.ack_path admite como máximo 2");
    // Más de 63 caracteres: "demasiado largo".
    expect_err("{\"aprs\":{\"path\":\"AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1\"}}",
               "demasiado largo");
}

TEST(ip_section)
{
    expect_ok("{\"ip\":{\"enabled\":true,\"mtu\":1}}");
    expect_ok("{\"ip\":{\"mtu\":65535,\"remote\":{\"addr\":\"44.1.2.3\"}}}");
    expect_err("{\"ip\":{\"mtu\":0}}", "ip.mtu debe estar entre 1 y 65535");
    expect_err("{\"ip\":{\"mtu\":65536}}", "ip.mtu");
    expect_err("{\"ip\":{\"enabled\":\"yes\"}}", "ip.enabled debe ser true o false");
    expect_err("{\"ip\":{\"remote\":\"x\"}}", "ip.remote debe ser un objeto");
    expect_err("{\"ip\":{\"remote\":{\"addr\":5}}}", "ip.remote.addr");
}

TEST(wifi_array)
{
    expect_ok("{\"wifi\":[]}");
    expect_ok("{\"wifi\":[{\"ssid\":\"Casa\",\"password\":\"x\",\"connect_timeout_s\":10}]}");
    expect_ok("{\"wifi\":[{\"ssid\":\"Casa\"},{\"ssid\":\"Movil\",\"password\":\"\"}]}");
    expect_err("{\"wifi\":[5]}", "wifi[0] debe ser un objeto");
    expect_err("{\"wifi\":[{\"ssid\":\"Casa\"},{\"ssid\":\"\"}]}", "wifi[1].ssid");
    expect_err("{\"wifi\":[{\"password\":\"x\"}]}", "wifi[0].ssid");
    expect_err("{\"wifi\":[{\"ssid\":5}]}", "wifi[0].ssid");
    expect_err("{\"wifi\":[{\"ssid\":\"A\",\"password\":5}]}", "wifi.password debe ser una cadena");
    expect_err("{\"wifi\":[{\"ssid\":\"A\",\"connect_timeout_s\":\"10\"}]}", "wifi.connect_timeout_s");
}

TEST(digi_section)
{
    expect_ok("{\"digi\":{\"enabled\":true,\"alias\":[\"WIDE1-1\",\"WIDE2-2\"],\"callsign\":\"RPT1\",\"ssid\":0}}");
    expect_ok("{\"digi\":{\"enabled\":false,\"alias\":[],\"comment\":\"x\",\"callsign\":\"\",\"ssid\":3}}");
    expect_err("{\"digi\":{\"enabled\":1}}", "digi.enabled debe ser true o false");
    expect_err("{\"digi\":{\"alias\":[1]}}", "digi.alias[0] debe ser una cadena");
    expect_err("{\"digi\":{\"alias\":[\"WIDE1-1\",null]}}", "digi.alias[1]");
    // Observación: digi_init acepta alias como cadena suelta, pero aquí se rechaza.
    expect_err("{\"digi\":{\"alias\":\"WIDE1-1\"}}", "digi.alias debe ser un array");
    expect_err("{\"digi\":{\"ssid\":\"0\"}}", "digi.ssid debe ser un número");
}

TEST(gps_ranges)
{
    expect_ok("{\"gps\":{\"beacon_period_s\":10}}");
    expect_ok("{\"gps\":{\"beacon_period_s\":86400,\"fixed_lat\":-90,\"fixed_lon\":-180}}");
    expect_ok("{\"gps\":{\"fixed_lat\":90,\"fixed_lon\":180}}");
    expect_err("{\"gps\":{\"beacon_period_s\":9}}", "gps.beacon_period_s debe estar entre 10 y 86400");
    expect_err("{\"gps\":{\"beacon_period_s\":86401}}", "gps.beacon_period_s");
    expect_err("{\"gps\":{\"fixed_lat\":90.5}}", "gps.fixed_lat debe estar entre -90 y 90");
    expect_err("{\"gps\":{\"fixed_lon\":181}}", "gps.fixed_lon");
    expect_err("{\"gps\":{\"altitude\":1}}", "gps.altitude debe ser true o false");
    expect_err("{\"gps\":{\"baud\":\"9600\"}}", "gps.baud debe ser un número");
    expect_err("{\"gps\":{\"smart_beacon\":\"x\"}}", "gps.smart_beacon debe ser un objeto");
    expect_ok("{\"gps\":{\"smart_beacon\":{\"enabled\":true,\"fast_speed_kmh\":100}}}");
    expect_err("{\"gps\":{\"smart_beacon\":{\"fast_rate_s\":\"a\"}}}", "gps.smart_beacon.fast_rate_s");
    expect_err("{\"gps\":{\"smart_beacon\":{\"enabled\":\"si\"}}}", "gps.smart_beacon.enabled");
}

TEST(console_tx_aprsmsg_ranges)
{
    expect_ok("{\"console\":{\"port\":1}}");
    expect_ok("{\"console\":{\"port\":65535}}");
    expect_err("{\"console\":{\"port\":0}}", "console.port debe estar entre 1 y 65535");
    expect_err("{\"console\":{\"port\":65536}}", "console.port");
    expect_err("{\"console\":{\"tcp\":\"si\"}}", "console.tcp");

    expect_ok("{\"tx\":{\"persist\":0,\"slottime_ms\":0,\"lbt_max_wait_ms\":0}}");
    expect_ok("{\"tx\":{\"persist\":255,\"slottime_ms\":2000,\"lbt_max_wait_ms\":60000}}");
    expect_err("{\"tx\":{\"persist\":256}}", "tx.persist debe estar entre 0 y 255");
    expect_err("{\"tx\":{\"slottime_ms\":-1}}", "tx.slottime_ms");
    expect_err("{\"tx\":{\"slottime_ms\":2001}}", "tx.slottime_ms");
    expect_err("{\"tx\":{\"lbt_max_wait_ms\":60001}}", "tx.lbt_max_wait_ms");
    expect_err("{\"tx\":{\"lbt_enabled\":1}}", "tx.lbt_enabled debe ser true o false");

    expect_ok("{\"aprs_msg\":{\"max_stored\":1}}");
    expect_ok("{\"aprs_msg\":{\"max_stored\":250}}");
    expect_err("{\"aprs_msg\":{\"max_stored\":0}}", "aprs_msg.max_stored debe estar entre 1 y 250");
    expect_err("{\"aprs_msg\":{\"max_stored\":251}}", "aprs_msg.max_stored");
}

TEST(other_sections_types)
{
    expect_ok("{\"ap\":{\"enabled\":true,\"ssid\":\"APRS\",\"password\":\"12345678\","
              "\"offer_gateway\":false,\"captive_portal\":true}}");
    expect_err("{\"ap\":{\"enabled\":\"no\"}}", "ap.enabled debe ser true o false");
    expect_err("{\"ap\":{\"offer_gateway\":0}}", "ap.offer_gateway");
    expect_err("{\"il2p\":{\"tx_all\":0}}", "il2p.tx_all debe ser true o false");
    expect_err("{\"display\":{\"i2c_addr\":\"0x3C\"}}", "display.i2c_addr debe ser un número");
    expect_err("{\"morse\":{\"wpm\":\"20\"}}", "morse.wpm debe ser un número");
    expect_err("{\"morse\":{\"callsign\":5}}", "morse.callsign");
    expect_err("{\"remote_cmd\":{\"enabled\":\"x\"}}", "remote_cmd.enabled");
    expect_err("{\"rx\":{\"squelch_threshold\":\"5\"}}", "rx.squelch_threshold debe ser un número");
    expect_err("{\"rx\":{\"dual_modem_enabled\":\"true\"}}", "rx.dual_modem_enabled debe ser true o false");
    expect_err("{\"rx\":{\"active_modem\":\"original\"}}", "rx.active_modem debe ser v2, v3 o best");
    expect_err("{\"rx\":{\"active_modem\":3}}", "rx.active_modem debe ser una cadena");
    expect_err("{\"repeater\":{\"cw_id\":1}}", "repeater.cw_id");
    expect_err("{\"squelch_sf\":{\"hfne_threshold\":\"x\"}}", "squelch_sf.hfne_threshold");
}

TEST(error_buffer_edge_cases)
{
    cJSON *bad = cJSON_Parse("{\"aprs\":{\"ssid\":99}}");
    cJSON *good = cJSON_Parse("{\"aprs\":{\"ssid\":1}}");
    char err[ERRBUF];

    // Buffer de longitud 1: sólo puede quedar la cadena vacía.
    memset(err, 'Z', sizeof(err));
    CHECK(!config_validate(bad, err, 1));
    CHECK_EQ_INT(err[0], '\0');

    // Longitud 0: no se escribe nada y no hay fallo.
    memset(err, 'Z', sizeof(err));
    CHECK(!config_validate(bad, err, 0));
    CHECK_EQ_INT(err[0], 'Z');

    // Puntero NULL con longitud > 0: no debe escribir ni fallar.
    CHECK(!config_validate(bad, NULL, 64));
    CHECK(!config_validate(bad, NULL, 0));

    // Mensaje truncado a la longitud dada, siempre terminado en NUL.
    CHECK(!config_validate(bad, err, 8));
    CHECK(strlen(err) <= 7);
    CHECK(strncmp(err, "aprs.ss", 7) == 0);

    // Éxito: el buffer se vacía aunque tuviera basura.
    strcpy(err, "basura");
    CHECK(config_validate(good, err, sizeof(err)));
    CHECK_STR(err, "");
    strcpy(err, "basura");
    CHECK(config_validate(good, err, 1));
    CHECK_EQ_INT(err[0], '\0');

    // Nulos con cualquier longitud.
    CHECK(!config_validate(NULL, NULL, 0));
    CHECK(!config_validate(NULL, err, 1));
    CHECK_EQ_INT(err[0], '\0');

    cJSON_Delete(bad);
    cJSON_Delete(good);
}

TEST(message_length_all_errors)
{
    // Todos los errores de la tabla anterior caben en 120 caracteres y no están vacíos.
    const char *bad[] = {
        "{\"aprs\":{\"path\":\"AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1,AAAAAA-1\"}}",
        "{\"digi\":{\"alias\":[1]}}",
        "{\"wifi\":[{\"ssid\":\"A\",\"password\":5}]}",
        "{\"gps\":{\"smart_beacon\":{\"fast_rate_s\":\"a\"}}}",
        "{\"ip\":{\"remote\":{\"addr\":5}}}",
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) expect_err(bad[i], NULL);
}

int main(void)
{
    RUN(defaults_template_is_valid);
    RUN(root_must_be_object);
    RUN(empty_and_unknown_keys_accepted);
    RUN(section_must_be_object);
    RUN(aprs_callsign_and_ssid);
    RUN(aprs_delay_range);
    RUN(aprs_path_formats);
    RUN(ip_section);
    RUN(wifi_array);
    RUN(digi_section);
    RUN(gps_ranges);
    RUN(console_tx_aprsmsg_ranges);
    RUN(other_sections_types);
    RUN(error_buffer_edge_cases);
    RUN(message_length_all_errors);
    return test_summary();
}
