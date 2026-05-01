#include "digipeater.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"   // afsk_queue_tx_frame, AX25_MAX_FRAME_LEN
#include "LibAPRS-esp32-i2s/src/AX25.h"
#include "LibAPRS-esp32-i2s/src/HDLC.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#define TAG "digi"

// ─── Configuration ────────────────────────────────────────────────────────────

#define MAX_ALIASES 8   // maximum number of path aliases supported

typedef struct { char call[8]; int ssid; } alias_t;

static volatile bool s_enabled       = false;
static alias_t       s_aliases[MAX_ALIASES];
static int           s_alias_count   = 0;
static char          s_call[8]       = "";
static int           s_call_ssid     = 0;

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

// Add one alias string to s_aliases[] if there is room.
static void alias_add(const char *str) {
    if (s_alias_count >= MAX_ALIASES) return;
    split_callssid(str, s_aliases[s_alias_count].call,
                        &s_aliases[s_alias_count].ssid);
    if (s_aliases[s_alias_count].call[0])
        s_alias_count++;
}

void digi_init(const cJSON *cfg) {
    if (!cfg) return;
    const cJSON *digi = cJSON_GetObjectItemCaseSensitive(cfg, "digi");
    if (!digi) return;

    const cJSON *en = cJSON_GetObjectItemCaseSensitive(digi, "enabled");
    s_enabled = cJSON_IsTrue(en);

    // "alias": accepts either a string ("WIDE1-1") for backward compat,
    // or a JSON array (["WIDE1-1","WIDE2-2","RELAY"]).
    s_alias_count = 0;
    const cJSON *alias_j = cJSON_GetObjectItemCaseSensitive(digi, "alias");
    if (cJSON_IsArray(alias_j)) {
        const cJSON *item;
        cJSON_ArrayForEach(item, alias_j) {
            if (cJSON_IsString(item) && item->valuestring[0])
                alias_add(item->valuestring);
        }
    } else if (cJSON_IsString(alias_j) && alias_j->valuestring[0]) {
        alias_add(alias_j->valuestring);
    }

    // Digipeater own callsign: falls back to aprs.callsign if not set
    const cJSON *cs_j = cJSON_GetObjectItemCaseSensitive(digi, "callsign");
    if (cJSON_IsString(cs_j) && cs_j->valuestring[0]) {
        const cJSON *ss_j = cJSON_GetObjectItemCaseSensitive(digi, "ssid");
        strncpy(s_call, cs_j->valuestring, 7); s_call[7] = '\0';
        s_call_ssid = cJSON_IsNumber(ss_j) ? (int)ss_j->valueint : 0;
    } else {
        const cJSON *aprs = cJSON_GetObjectItemCaseSensitive(cfg, "aprs");
        if (aprs) {
            const cJSON *ac = cJSON_GetObjectItemCaseSensitive(aprs, "callsign");
            const cJSON *as = cJSON_GetObjectItemCaseSensitive(aprs, "ssid");
            if (cJSON_IsString(ac)) { strncpy(s_call, ac->valuestring, 7); s_call[7] = '\0'; }
            if (cJSON_IsNumber(as)) s_call_ssid = (int)as->valueint;
        }
    }

    ESP_LOGI(TAG, "digi_init: enabled=%d aliases=%d via=%s-%d",
             s_enabled, s_alias_count, s_call, s_call_ssid);
    for (int i = 0; i < s_alias_count; i++)
        ESP_LOGI(TAG, "  alias[%d]: %s-%d", i, s_aliases[i].call, s_aliases[i].ssid);
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

typedef struct {
    uint32_t hash;
    int64_t  ts_us;   // esp_timer_get_time() at insertion
} dedup_entry_t;

static dedup_entry_t s_dedup[DEDUP_SLOTS];
static int           s_dedup_next = 0;   // next slot to overwrite (ring)

// FNV-1a 32-bit hash of arbitrary byte sequence.
static uint32_t fnv1a(const uint8_t *data, size_t len) {
    uint32_t h = 0x811c9dc5u;
    for (size_t i = 0; i < len; i++)
        h = (h ^ data[i]) * 0x01000193u;
    return h;
}

// Returns true if this hash was seen within DEDUP_TTL_US; otherwise records it.
static bool dedup_check_and_add(uint32_t hash) {
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < DEDUP_SLOTS; i++) {
        if (s_dedup[i].hash == hash &&
            (now - s_dedup[i].ts_us) < DEDUP_TTL_US) {
            return true;   // duplicate
        }
    }
    // Not found (or expired) — record in ring slot and return false
    s_dedup[s_dedup_next].hash   = hash;
    s_dedup[s_dedup_next].ts_us  = now;
    s_dedup_next = (s_dedup_next + 1) % DEDUP_SLOTS;
    return false;
}

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

bool digi_process_frame(const uint8_t *buf, size_t len) {
    if (!s_enabled) return false;
    if (len < 16 || s_call[0] == '\0' || s_alias_count == 0) return false;

    // ── Duplicate check (on the original frame before we modify it) ──────────
    // Hash covers DST+SRC+INFO: enough to identify unique packets.
    // Path is deliberately excluded so we match the same content regardless of
    // how many hops it has already traversed before reaching us.
    // We hash: buf[0..13] (DST+SRC) + everything after CTRL+PID.
    // Simple approach: hash the entire raw frame — same frame from two
    // directions will have different paths and NOT be suppressed, which is
    // correct for multi-path environments. We want to suppress only if WE
    // already retransmitted THIS exact received frame (same buf bytes).
    uint32_t frame_hash = fnv1a(buf, len);
    if (dedup_check_and_add(frame_hash)) {
        ESP_LOGD(TAG, "Duplicate frame suppressed (hash=0x%08" PRIx32 ")", frame_hash);
        return false;
    }

    // ── Work on a mutable copy ───────────────────────────────────────────────
    static uint8_t frame[AX25_MAX_FRAME_LEN];
    if (len > AX25_MAX_FRAME_LEN) return false;
    memcpy(frame, buf, len);

    uint8_t *p   = frame;
    uint8_t *end = frame + len;

    // Skip DST and SRC
    if (p + 14 > end) return false;
    bool is_last_src = (p[13] & 0x01) != 0;   // SRC end-bit at byte 13
    p += 14;

    // ── Loop-prevention: bail if our callsign (H=1) already in path ─────────
    {
        const uint8_t *scan = p;
        bool scan_last = is_last_src;
        while (!scan_last && (scan + 7) <= end) {
            char sc[8]; int ss; bool sh, sl;
            addr_read(scan, sc, &ss, &sh, &sl);
            if (sh && call_eq(sc, s_call) && ss == s_call_ssid) {
                return false;   // we already repeated this frame
            }
            scan_last = sl;
            scan += 7;
        }
    }

    // ── Find first unused path entry matching any of our aliases ─────────────
    bool did_digi = false;
    bool cur_last = is_last_src;

    while (!cur_last && (p + 7) <= end) {
        char rpt_call[8]; int rpt_ssid; bool rpt_h, rpt_last;
        addr_read(p, rpt_call, &rpt_ssid, &rpt_h, &rpt_last);

        if (!rpt_h) {
            // Check against every configured alias
            for (int ai = 0; ai < s_alias_count; ai++) {
                if (!call_eq(rpt_call, s_aliases[ai].call)) continue;
                if (rpt_ssid != s_aliases[ai].ssid) continue;

                // Match found — apply standard WIDEn-N digipeat:
                // 1. Insert our callsign (H=1) before this entry (traceback).
                // 2. Decrement N; if N hits 0 mark alias as spent (H=1).
                int new_ssid = rpt_ssid - 1;

                if (len + 7 > AX25_MAX_FRAME_LEN) break;   // no room

                memmove(p + 7, p, (size_t)(end - p));
                len += 7;
                end += 7;

                addr_write(p, s_call, s_call_ssid, /*last=*/false, /*h=*/true);
                p += 7;

                addr_write(p, rpt_call, new_ssid, rpt_last, /*h=*/(new_ssid == 0));

                char src_call[8]; int src_ssid; bool src_h, src_last;
                addr_read(&frame[7], src_call, &src_ssid, &src_h, &src_last);
                ESP_LOGI(TAG, "%s-%d digipeated via alias %s-%d → %s-%d (len=%u)",
                         src_call, src_ssid,
                         s_aliases[ai].call, s_aliases[ai].ssid,
                         s_call, s_call_ssid, (unsigned)len);
                did_digi = true;
                break;
            }
        }

        if (did_digi) break;   // one digipeat per frame
        cur_last = rpt_last;
        p += 7;
    }

    if (did_digi)
        afsk_queue_tx_frame(frame, len);

    return did_digi;
}
