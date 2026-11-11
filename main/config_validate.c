// Validación de config.json recibida por POST /api/config (sin heap, sólo pila).
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config_validate.h"

// Contexto de error: buffer del llamador. Los mensajes se truncan a err_len.
typedef struct {
    char  *err;
    size_t len;
} ctx_t;

// Escribe el mensaje de error y devuelve false (para usar en `return fail(...)`).
static bool fail(const ctx_t *c, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static bool fail(const ctx_t *c, const char *fmt, ...)
{
    if (c->err && c->len > 0) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(c->err, c->len, fmt, ap);
        va_end(ap);
    }
    return false;
}

// Clave de un objeto (sensible a mayúsculas, como el resto del firmware). NULL si no existe.
static const cJSON *item(const cJSON *obj, const char *key)
{
    return cJSON_GetObjectItemCaseSensitive(obj, key);
}

// ---- Comprobaciones de tipo y rango (si la clave no existe, no hay error) ----

static bool t_bool(const ctx_t *c, const cJSON *o, const char *sec, const char *key)
{
    const cJSON *it = item(o, key);
    if (it && !cJSON_IsBool(it)) return fail(c, "%s.%s debe ser true o false", sec, key);
    return true;
}

static bool t_num(const ctx_t *c, const cJSON *o, const char *sec, const char *key)
{
    const cJSON *it = item(o, key);
    if (it && !cJSON_IsNumber(it)) return fail(c, "%s.%s debe ser un número", sec, key);
    return true;
}

static bool r_num(const ctx_t *c, const cJSON *o, const char *sec, const char *key,
                  double min, double max)
{
    if (!t_num(c, o, sec, key)) return false;
    const cJSON *it = item(o, key);
    if (it && (it->valuedouble < min || it->valuedouble > max))
        return fail(c, "%s.%s debe estar entre %g y %g", sec, key, min, max);
    return true;
}

static bool t_str(const ctx_t *c, const cJSON *o, const char *sec, const char *key)
{
    const cJSON *it = item(o, key);
    if (it && !cJSON_IsString(it)) return fail(c, "%s.%s debe ser una cadena de texto", sec, key);
    return true;
}

static bool t_arr(const ctx_t *c, const cJSON *o, const char *sec, const char *key)
{
    const cJSON *it = item(o, key);
    if (it && !cJSON_IsArray(it)) return fail(c, "%s.%s debe ser un array", sec, key);
    return true;
}

static bool t_obj(const ctx_t *c, const cJSON *o, const char *sec, const char *key)
{
    const cJSON *it = item(o, key);
    if (it && !cJSON_IsObject(it)) return fail(c, "%s.%s debe ser un objeto", sec, key);
    return true;
}

// ---- Formatos específicos ----

static bool is_upper_alnum(char ch)
{
    return (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9');
}

// Indicativo: 1–6 caracteres alfanuméricos en mayúsculas.
static bool valid_call(const char *s, size_t n)
{
    if (n < 1 || n > 6) return false;
    for (size_t i = 0; i < n; i++)
        if (!is_upper_alnum(s[i])) return false;
    return true;
}

// Path de digipeat: "" o hasta 2 repetidores CALL[-N] separados por coma (N 0–15).
// Se admiten espacios alrededor de cada entrada, como hace el parser de LibAPRS.
static bool check_path(const ctx_t *c, const char *key, const char *val)
{
    char buf[64];
    size_t len = strlen(val);
    if (len >= sizeof(buf)) return fail(c, "%s demasiado largo (máx. 2 repetidores)", key);
    memcpy(buf, val, len + 1);

    char *t = buf;
    while (*t == ' ') t++;
    size_t tl = strlen(t);
    while (tl > 0 && t[tl - 1] == ' ') t[--tl] = '\0';
    if (*t == '\0') return true;   // sin repetidores

    int count = 0;
    char *tok = t;
    while (tok) {
        char *comma = strchr(tok, ',');
        if (comma) *comma = '\0';

        char *e = tok;
        while (*e == ' ') e++;
        size_t el = strlen(e);
        while (el > 0 && e[el - 1] == ' ') e[--el] = '\0';

        if (*e == '\0') return fail(c, "%s tiene un repetidor vacío (revisa las comas)", key);
        if (++count > 2) return fail(c, "%s admite como máximo 2 repetidores", key);

        char *dash = strchr(e, '-');
        size_t cl = dash ? (size_t)(dash - e) : el;
        if (!valid_call(e, cl))
            return fail(c, "%s: indicativo '%.12s' no válido (1-6 alfanuméricos en mayúsculas)", key, e);

        if (dash) {
            const char *d = dash + 1;
            size_t dl = strlen(d);
            if (dl < 1 || dl > 2 || d[0] < '0' || d[0] > '9' ||
                (dl == 2 && (d[1] < '0' || d[1] > '9')))
                return fail(c, "%s: SSID de '%.12s' no válido (0-15)", key, e);
            if (atoi(d) > 15) return fail(c, "%s: SSID de '%.12s' fuera de 0-15", key, e);
        }

        if (!comma) break;
        tok = comma + 1;
    }
    return true;
}

// ---- Secciones (cada una recibe el objeto ya comprobado como tal) ----

static bool check_aprs(const ctx_t *c, const cJSON *o)
{
    if (!t_str(c, o, "aprs", "callsign") || !t_num(c, o, "aprs", "ssid") ||
        !t_str(c, o, "aprs", "symbol_table") || !t_str(c, o, "aprs", "symbol_code") ||
        !r_num(c, o, "aprs", "post_rx_tx_delay_ms", 0, 5000) ||
        !t_str(c, o, "aprs", "path") || !t_str(c, o, "aprs", "ack_path"))
        return false;

    const cJSON *cs = item(o, "callsign");
    if (cs && !valid_call(cs->valuestring, strlen(cs->valuestring)))
        return fail(c, "aprs.callsign debe tener 1-6 caracteres alfanuméricos en mayúsculas");

    const cJSON *ssid = item(o, "ssid");
    if (ssid && (ssid->valuedouble < 0 || ssid->valuedouble > 15))
        return fail(c, "aprs.ssid debe estar entre 0 y 15");

    const cJSON *p = item(o, "path");
    if (p && !check_path(c, "aprs.path", p->valuestring)) return false;
    const cJSON *ap = item(o, "ack_path");
    if (ap && !check_path(c, "aprs.ack_path", ap->valuestring)) return false;
    return true;
}

static bool check_wifi(const ctx_t *c, const cJSON *arr)
{
    int idx = 0;
    for (const cJSON *n = arr->child; n; n = n->next, idx++) {
        if (!cJSON_IsObject(n)) return fail(c, "wifi[%d] debe ser un objeto", idx);
        const cJSON *ssid = item(n, "ssid");
        if (!ssid || !cJSON_IsString(ssid) || ssid->valuestring[0] == '\0')
            return fail(c, "wifi[%d].ssid debe ser una cadena no vacía", idx);
        if (!t_str(c, n, "wifi", "password") || !t_num(c, n, "wifi", "connect_timeout_s"))
            return false;
    }
    return true;
}

static bool check_ap(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "ap", "enabled") && t_str(c, o, "ap", "ssid") &&
           t_str(c, o, "ap", "password") && t_bool(c, o, "ap", "offer_gateway") &&
           t_bool(c, o, "ap", "captive_portal");
}

static bool check_ip(const ctx_t *c, const cJSON *o)
{
    if (!t_bool(c, o, "ip", "enabled") || !t_str(c, o, "ip", "mode") ||
        !r_num(c, o, "ip", "mtu", 1, 65535) ||
        !t_str(c, o, "ip", "addr") || !t_str(c, o, "ip", "netmask") ||
        !t_str(c, o, "ip", "gateway") || !t_str(c, o, "ip", "remote_addr") ||
        !t_obj(c, o, "ip", "remote"))
        return false;
    const cJSON *rem = item(o, "remote");
    return !rem || t_str(c, rem, "ip.remote", "addr");
}

static bool check_il2p(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "il2p", "enabled") && t_bool(c, o, "il2p", "max_fec") &&
           t_bool(c, o, "il2p", "tx_all");
}

static bool check_digi(const ctx_t *c, const cJSON *o)
{
    if (!t_bool(c, o, "digi", "enabled") || !t_arr(c, o, "digi", "alias") ||
        !t_str(c, o, "digi", "comment") || !t_str(c, o, "digi", "callsign") ||
        !t_num(c, o, "digi", "ssid"))
        return false;
    const cJSON *al = item(o, "alias");
    if (!al) return true;
    int idx = 0;
    for (const cJSON *n = al->child; n; n = n->next, idx++)
        if (!cJSON_IsString(n)) return fail(c, "digi.alias[%d] debe ser una cadena de texto", idx);
    return true;
}

static bool check_morse(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "morse", "enabled") && t_num(c, o, "morse", "tone_hz") &&
           t_num(c, o, "morse", "wpm") && t_num(c, o, "morse", "period_s") &&
           t_str(c, o, "morse", "callsign");
}

static bool check_smart_beacon(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "gps.smart_beacon", "enabled") &&
           t_num(c, o, "gps.smart_beacon", "fast_speed_kmh") &&
           t_num(c, o, "gps.smart_beacon", "slow_speed_kmh") &&
           t_num(c, o, "gps.smart_beacon", "fast_rate_s") &&
           t_num(c, o, "gps.smart_beacon", "slow_rate_s") &&
           t_num(c, o, "gps.smart_beacon", "min_turn_deg") &&
           t_num(c, o, "gps.smart_beacon", "turn_slope") &&
           t_num(c, o, "gps.smart_beacon", "min_turn_time_s");
}

static bool check_gps(const ctx_t *c, const cJSON *o)
{
    if (!t_bool(c, o, "gps", "enabled") || !t_num(c, o, "gps", "baud") ||
        !t_bool(c, o, "gps", "use_for_beacon") ||
        !r_num(c, o, "gps", "beacon_period_s", 10, 86400) ||
        !t_bool(c, o, "gps", "altitude") || !t_bool(c, o, "gps", "use_fixed") ||
        !r_num(c, o, "gps", "fixed_lat", -90, 90) ||
        !r_num(c, o, "gps", "fixed_lon", -180, 180) ||
        !t_str(c, o, "gps", "symbol_table") || !t_str(c, o, "gps", "symbol_code") ||
        !t_str(c, o, "gps", "comment") || !t_obj(c, o, "gps", "smart_beacon"))
        return false;
    const cJSON *sb = item(o, "smart_beacon");
    return !sb || check_smart_beacon(c, sb);
}

static bool check_display(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "display", "enabled") && t_num(c, o, "display", "i2c_addr");
}

static bool check_rx(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "rx", "dual_modem_enabled") && t_str(c, o, "rx", "active_modem") &&
           t_num(c, o, "rx", "squelch_threshold") && t_bool(c, o, "rx", "deemphasis_enabled");
}

static bool check_remote_cmd(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "remote_cmd", "enabled");
}

static bool check_console(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "console", "enabled") && t_bool(c, o, "console", "tcp") &&
           t_bool(c, o, "console", "udp") && r_num(c, o, "console", "port", 1, 65535);
}

static bool check_repeater(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "repeater", "enabled") &&
           t_num(c, o, "repeater", "squelch_threshold") &&
           t_num(c, o, "repeater", "lockout_ms") && t_num(c, o, "repeater", "max_record_s") &&
           t_num(c, o, "repeater", "tail_delay_ms") &&
           t_num(c, o, "repeater", "courtesy_tone_hz") &&
           t_num(c, o, "repeater", "courtesy_tone_ms") && t_bool(c, o, "repeater", "cw_id");
}

static bool check_tx(const ctx_t *c, const cJSON *o)
{
    return t_bool(c, o, "tx", "lbt_enabled") &&
           r_num(c, o, "tx", "lbt_max_wait_ms", 0, 60000) &&
           r_num(c, o, "tx", "persist", 0, 255) &&
           r_num(c, o, "tx", "slottime_ms", 0, 2000);
}

static bool check_squelch_sf(const ctx_t *c, const cJSON *o)
{
    return t_num(c, o, "squelch_sf", "hfne_threshold");
}

static bool check_aprs_msg(const ctx_t *c, const cJSON *o)
{
    return r_num(c, o, "aprs_msg", "max_stored", 1, 250);
}

// Tabla de secciones conocidas: nombre de la clave en la raíz y su comprobación.
typedef bool (*section_fn_t)(const ctx_t *c, const cJSON *o);
typedef struct {
    const char  *name;
    section_fn_t fn;
} section_t;

static const section_t SECTIONS[] = {
    { "aprs",       check_aprs },
    { "ap",         check_ap },
    { "ip",         check_ip },
    { "il2p",       check_il2p },
    { "digi",       check_digi },
    { "morse",      check_morse },
    { "gps",        check_gps },
    { "display",    check_display },
    { "rx",         check_rx },
    { "remote_cmd", check_remote_cmd },
    { "console",    check_console },
    { "repeater",   check_repeater },
    { "tx",         check_tx },
    { "squelch_sf", check_squelch_sf },
    { "aprs_msg",   check_aprs_msg },
};

bool config_validate(const cJSON *cfg, char *err, size_t err_len)
{
    ctx_t c = { err, err_len };

    if (err && err_len > 0) err[0] = '\0';
    if (!cfg || !cJSON_IsObject(cfg))
        return fail(&c, "la configuración debe ser un objeto JSON");

    const cJSON *ver = item(cfg, "version");
    if (ver && !cJSON_IsString(ver)) return fail(&c, "version debe ser una cadena de texto");

    // Secciones conocidas: si existen, deben ser objetos y cumplir sus reglas.
    for (size_t i = 0; i < sizeof(SECTIONS) / sizeof(SECTIONS[0]); i++) {
        const cJSON *s = item(cfg, SECTIONS[i].name);
        if (!s) continue;
        if (!cJSON_IsObject(s)) return fail(&c, "%s debe ser un objeto", SECTIONS[i].name);
        if (!SECTIONS[i].fn(&c, s)) return false;
    }

    // wifi: array de objetos con ssid no vacío.
    const cJSON *wifi = item(cfg, "wifi");
    if (wifi) {
        if (!cJSON_IsArray(wifi)) return fail(&c, "wifi debe ser un array");
        if (!check_wifi(&c, wifi)) return false;
    }

    return true;
}
