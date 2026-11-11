#include "digipeater.h"
#include "dedup.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"   // afsk_queue_tx_frame, AX25_MAX_FRAME_LEN
#include "LibAPRS-esp32-i2s/src/AX25.h"
#include "LibAPRS-esp32-i2s/src/HDLC.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#define TAG "digi"

// ─── Configuration ────────────────────────────────────────────────────────────

#define MAX_ALIASES 8   // maximum number of path aliases supported

typedef struct { char call[8]; int ssid; } alias_t;

static volatile bool   s_enabled       = false;
static alias_t         s_aliases[MAX_ALIASES];
static int             s_alias_count   = 0;
static char            s_call[8]       = "";
static int             s_call_ssid     = 0;
static char            s_my_call[8]    = "";   // aprs.callsign/ssid (origen propio)
static int             s_my_ssid       = 0;

// Protects config fields written by digi_init (httpd task) and read by
// digi_process_frame (receive_audio_task). portMUX is a spinlock safe for
// cross-core use without disabling the scheduler.
static portMUX_TYPE s_digi_mux = portMUX_INITIALIZER_UNLOCKED;

// Parse "CALL" or "CALL-N" into alias_t.
static void split_callssid(const char *src, char *call_out, int *ssid_out) {
    const char *dash = strchr(src, '-');
    if (dash && *(dash + 1) != '\0') {
        size_t clen = (size_t)(dash - src);
        if (clen > 7) clen = 7;
        memcpy(call_out, src, clen);
        call_out[clen] = '\0';
        *ssid_out = atoi(dash + 1);
    } else {
        size_t clen = strlen(src);
        if (clen > 7) clen = 7;
        memcpy(call_out, src, clen);
        call_out[clen] = '\0';
        *ssid_out = 0;
    }
}

void digi_init(const cJSON *cfg) {
    if (!cfg) return;
    const cJSON *digi = cJSON_GetObjectItemCaseSensitive(cfg, "digi");
    if (!digi) return;

    // ── Parse into local variables (no lock needed; JSON read-only here) ──────
    bool new_enabled = false;
    alias_t new_aliases[MAX_ALIASES];
    int new_count = 0;
    char new_call[8] = "";
    int  new_ssid = 0;

    const cJSON *en = cJSON_GetObjectItemCaseSensitive(digi, "enabled");
    new_enabled = cJSON_IsTrue(en);

    const cJSON *alias_j = cJSON_GetObjectItemCaseSensitive(digi, "alias");
    // Helper lambda pattern not available in C; inline the alias parsing:
    if (cJSON_IsArray(alias_j)) {
        const cJSON *item;
        cJSON_ArrayForEach(item, alias_j) {
            if (new_count >= MAX_ALIASES) break;
            if (!cJSON_IsString(item) || !item->valuestring[0]) continue;
            split_callssid(item->valuestring,
                           new_aliases[new_count].call,
                           &new_aliases[new_count].ssid);
            if (new_aliases[new_count].call[0]) new_count++;
        }
    } else if (cJSON_IsString(alias_j) && alias_j->valuestring[0]) {
        split_callssid(alias_j->valuestring,
                       new_aliases[new_count].call,
                       &new_aliases[new_count].ssid);
        if (new_aliases[new_count].call[0]) new_count++;
    }

    const cJSON *cs_j = cJSON_GetObjectItemCaseSensitive(digi, "callsign");
    if (cJSON_IsString(cs_j) && cs_j->valuestring[0]) {
        const cJSON *ss_j = cJSON_GetObjectItemCaseSensitive(digi, "ssid");
        strncpy(new_call, cs_j->valuestring, 7); new_call[7] = '\0';
        new_ssid = cJSON_IsNumber(ss_j) ? (int)ss_j->valueint : 0;
    } else {
        const cJSON *aprs = cJSON_GetObjectItemCaseSensitive(cfg, "aprs");
        if (aprs) {
            const cJSON *ac = cJSON_GetObjectItemCaseSensitive(aprs, "callsign");
            const cJSON *as = cJSON_GetObjectItemCaseSensitive(aprs, "ssid");
            if (cJSON_IsString(ac)) { strncpy(new_call, ac->valuestring, 7); new_call[7] = '\0'; }
            if (cJSON_IsNumber(as)) new_ssid = (int)as->valueint;
        }
    }

    // Indicativo propio de aprs.* (para descartar tramas con origen propio)
    char my_call[8] = "";
    int  my_ssid = 0;
    {
        const cJSON *aprs = cJSON_GetObjectItemCaseSensitive(cfg, "aprs");
        if (aprs) {
            const cJSON *ac = cJSON_GetObjectItemCaseSensitive(aprs, "callsign");
            const cJSON *as = cJSON_GetObjectItemCaseSensitive(aprs, "ssid");
            if (cJSON_IsString(ac)) { strncpy(my_call, ac->valuestring, 7); my_call[7] = '\0'; }
            if (cJSON_IsNumber(as)) my_ssid = (int)as->valueint;
        }
    }

    // ── Apply atomically so digi_process_frame always sees a consistent view ──
    taskENTER_CRITICAL(&s_digi_mux);
    s_enabled     = new_enabled;
    s_alias_count = new_count;
    memcpy(s_aliases, new_aliases, (size_t)new_count * sizeof(alias_t));
    memcpy(s_call, new_call, 8);
    s_call_ssid   = new_ssid;
    memcpy(s_my_call, my_call, 8);
    s_my_ssid     = my_ssid;
    taskEXIT_CRITICAL(&s_digi_mux);

    ESP_LOGI(TAG, "digi_init: enabled=%d aliases=%d via=%s-%d",
             new_enabled, new_count, new_call, new_ssid);
    for (int i = 0; i < new_count; i++)
        ESP_LOGI(TAG, "  alias[%d]: %s-%d", i, new_aliases[i].call, new_aliases[i].ssid);
}

void digi_set_enabled(bool en) { s_enabled = en; }
bool digi_is_enabled(void)     { return s_enabled; }

// ─── Duplicate suppression ────────────────────────────────────────────────────
// Keep a rolling table of FNV-1a hashes of recently forwarded frames.
// A frame is suppressed if an identical hash was forwarded within DEDUP_TTL_US.
// Table size 32 is enough: at 1200 bps a single frame takes ~0.5 s to transmit,
// so the realistic rate of unique frames is well under 32 per 30 s.

#define DEDUP_SLOTS   32
#define DEDUP_TTL_US  (30LL * 1000000LL)   // 30 seconds in microseconds

static dedup_entry_t s_dedup_entries[DEDUP_SLOTS];
static dedup_t       s_dedup = { s_dedup_entries, DEDUP_SLOTS, 0, DEDUP_TTL_US };

// ─── AX.25 address helpers ───────────────────────────────────────────────────

static const uint8_t *addr_read(const uint8_t *p,
                                 char *call_out, int *ssid_out,
                                 bool *h_out, bool *is_last_out) {
    for (int i = 0; i < 6; i++) call_out[i] = (char)(p[i] >> 1);
    call_out[6] = '\0';
    for (int i = 5; i >= 0 && call_out[i] == ' '; i--) call_out[i] = '\0';
    *ssid_out    = (p[6] >> 1) & 0x0F;
    *h_out       = (p[6] & 0x80) != 0;
    *is_last_out = (p[6] & 0x01) != 0;
    return p + 7;
}

static void addr_write(uint8_t *p, const char *call, int ssid, bool last, bool h) {
    int i;
    for (i = 0; i < 6 && call[i]; i++) p[i] = (uint8_t)((uint8_t)call[i] << 1);
    for (; i < 6; i++) p[i] = (uint8_t)(' ' << 1);
    p[6] = (uint8_t)(((ssid & 0x0F) << 1) | (last ? 0x01 : 0x00)
                     | (h ? 0x80 : 0x00) | 0x60);
}

static bool call_eq(const char *a, const char *b) {
    for (;;) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca -= 32;
        if (cb >= 'a' && cb <= 'z') cb -= 32;
        if (ca != cb) return false;
        if (ca == '\0') return true;
        a++; b++;
    }
}

// ─── Main digipeat logic ──────────────────────────────────────────────────────

// Política: máximo N aceptado en WIDEn-N (el SSID del alias es el N restante).
// Estándar actual: WIDE1-1 / WIDE2-2; rechaza p. ej. WIDE7-7. Subir si se desea.
#define DIGI_MAX_N          2
// Límite AX.25 de repetidores en el path.
#define DIGI_MAX_PATH_RPT   8

// Buffer de trabajo estático (no en la pila de aprs_poll_task, 3072 B). Se usa
// primero para componer los datos del hash y luego como copia mutable. El
// callback no debe bloquear: si ya está en uso (reentrada desde otra tarea) la
// trama se descarta en lugar de esperar.
static uint8_t s_work[AX25_MAX_FRAME_LEN];
static volatile bool s_work_busy = false;

bool digi_process_frame(const uint8_t *buf, size_t len) {
    // Snapshot config atomically to prevent races with digi_init called from
    // the httpd task during a config reload.
    taskENTER_CRITICAL(&s_digi_mux);
    bool    cfg_enabled = s_enabled;
    int     cfg_count   = s_alias_count;
    alias_t cfg_aliases[MAX_ALIASES];
    memcpy(cfg_aliases, s_aliases, (size_t)cfg_count * sizeof(alias_t));
    char cfg_call[8];
    memcpy(cfg_call, s_call, 8);
    int cfg_ssid = s_call_ssid;
    char my_call[8];
    memcpy(my_call, s_my_call, 8);
    int my_ssid = s_my_ssid;
    taskEXIT_CRITICAL(&s_digi_mux);

    if (!cfg_enabled) return false;
    if (len < 16 || len > AX25_MAX_FRAME_LEN || cfg_call[0] == '\0') return false;

    // ── Origen propio: no repetir nuestras balizas si vuelven por otro digi ──
    {
        char sc[8]; int ss; bool sh, sl;
        addr_read(buf + 7, sc, &ss, &sh, &sl);
        if ((call_eq(sc, cfg_call) && ss == cfg_ssid) ||
            (my_call[0] && call_eq(sc, my_call) && ss == my_ssid))
            return false;
    }

    // ── Localizar fin del campo de direcciones (bit 0 del SSID) ──────────────
    size_t addr_end = 14;
    bool is_last_src = (buf[13] & 0x01) != 0;   // SRC end-bit at byte 13
    int  path_n = 0;
    if (!is_last_src) {
        for (;;) {
            if (addr_end + 7 > len) return false;
            bool last = (buf[addr_end + 6] & 0x01) != 0;
            addr_end += 7;
            path_n++;
            if (last) break;
            if (path_n >= 8) return false;   // path malformado (>8 sin bit final)
        }
    }
    if (addr_end + 2 > len) return false;                 // falta control + PID
    size_t info_off = addr_end + 2;

    if (__atomic_exchange_n(&s_work_busy, true, __ATOMIC_ACQUIRE)) return false;
    bool did_digi = false;

    // ── Buscar el primer salto sin usar (H=0) del path ───────────────────────
    uint8_t *frame = s_work;
    memcpy(frame, buf, len);
    uint8_t *p   = frame + 14;
    uint8_t *end = frame + len;

    // Loop-prevention: si nuestro indicativo ya está con H=1 en el path, salir.
    for (int i = 0, n = path_n; i < n; i++) {
        char sc[8]; int ss; bool sh, sl;
        addr_read(p + 7 * i, sc, &ss, &sh, &sl);
        if (sh && call_eq(sc, cfg_call) && ss == cfg_ssid) goto out;
    }

    {
        bool cur_last = is_last_src;
        while (!cur_last && (p + 7) <= end) {
            char rpt_call[8]; int rpt_ssid; bool rpt_h, rpt_last;
            addr_read(p, rpt_call, &rpt_ssid, &rpt_h, &rpt_last);
            if (rpt_h) {   // ya repetido por otro: pasar al primer salto pendiente
                cur_last = rpt_last;
                p += 7;
                continue;
            }

            // p es el primer salto sin usar: es el único sobre el que se puede
            // actuar (AX.25). Dos casos: indicativo propio explícito o alias.
            bool explicit_me = call_eq(rpt_call, cfg_call) && rpt_ssid == cfg_ssid;
            int  ai_match = -1;
            if (!explicit_me) {
                for (int ai = 0; ai < cfg_count; ai++) {
                    // B24: alias con SSID N (p. ej. WIDE2-2) acepta WIDE2-1..N (salto restante);
                    // alias sin SSID (RELAY) sólo casa sin SSID.
                    if (!call_eq(rpt_call, cfg_aliases[ai].call)) continue;
                    int as = cfg_aliases[ai].ssid;
                    if ((as == 0 && rpt_ssid == 0) || (as > 0 && rpt_ssid >= 1 && rpt_ssid <= as)) {
                        ai_match = ai; break;
                    }
                }
                if (ai_match < 0) goto out;
                if (rpt_ssid > DIGI_MAX_N) goto out;     // WIDE7-7 etc.
                if (path_n >= DIGI_MAX_PATH_RPT) goto out;   // sin hueco para insertar
                if (len + 7 > AX25_MAX_FRAME_LEN) goto out;
            }

            // ── Dedup sobre origen + destino + información (sin path) ────────
            // Se comprueba aquí (sólo si vamos a repetir) para no "quemar" el
            // hash con tramas que no nos corresponden. Se compone en s_work
            // ([dst][src][info], con SSID enmascarado) y luego se restaura la copia.
            size_t ilen = len - info_off;
            size_t poff = (size_t)(p - frame);        // posición del salto
            memcpy(frame, buf, 14);
            frame[6]  &= 0x1E;    // sólo SSID (sin H, bit final ni reservados)
            frame[13] &= 0x1E;
            memmove(frame + 14, buf + info_off, ilen);
            uint32_t frame_hash = dedup_hash(frame, 14 + ilen);
            if (dedup_check_and_add(&s_dedup, frame_hash)) {
                ESP_LOGD(TAG, "Duplicate frame suppressed (hash=0x%08" PRIx32 ")", frame_hash);
                goto out;
            }

            // Restaurar la copia mutable y reposicionar punteros.
            memcpy(frame, buf, len);
            p   = frame + poff;
            end = frame + len;

            if (explicit_me) {
                // Indicativo propio en el path: sólo se marca H=1 (sin insertar).
                p[6] |= 0x80;
                ESP_LOGI(TAG, "digipeated via own callsign %s-%d (len=%u)",
                         cfg_call, cfg_ssid, (unsigned)len);
            } else {
                // rpt_ssid>0 aquí salvo alias "WIDE0-0": clamp a 0 (si no, -1
                // daría 15 al empaquetar y el H quedaría sin marcar).
                int new_ssid = (rpt_ssid > 0) ? rpt_ssid - 1 : 0;
                memmove(p + 7, p, (size_t)(end - p));
                len += 7;
                addr_write(p, cfg_call, cfg_ssid, /*last=*/false, /*h=*/true);
                p += 7;
                addr_write(p, rpt_call, new_ssid, rpt_last, /*h=*/(new_ssid == 0));
                ESP_LOGI(TAG, "digipeated via alias %s-%d -> %s-%d (len=%u)",
                         cfg_aliases[ai_match].call, cfg_aliases[ai_match].ssid,
                         cfg_call, cfg_ssid, (unsigned)len);
            }
            did_digi = true;
            break;
        }
    }

    if (did_digi)
        afsk_queue_tx_frame(frame, len);   // copia a la cola (no retiene el puntero)

out:
    __atomic_store_n(&s_work_busy, false, __ATOMIC_RELEASE);
    return did_digi;
}
