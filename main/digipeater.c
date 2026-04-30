#include "digipeater.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"   // afsk_queue_tx_frame
#include "LibAPRS-esp32-i2s/src/AX25.h"   // AX25_CTRL_UI, AX25_PID_NOLAYER3
#include "LibAPRS-esp32-i2s/src/HDLC.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#define TAG "digi"

// ─── Configuration ────────────────────────────────────────────────────────────

static volatile bool s_enabled   = false;
static char          s_alias[16] = "WIDE1";   // matched call (no SSID part)
static int           s_alias_ssid = 1;         // matched SSID
static char          s_call[8]   = "";         // our digipeater callsign
static int           s_call_ssid = 0;

// Parse a "CALL-SSID" string into separate fields.
static void split_callssid(const char *src, char *call_out, int *ssid_out) {
    const char *dash = strchr(src, '-');
    if (dash && *(dash+1) != '\0') {
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

    const cJSON *en = cJSON_GetObjectItemCaseSensitive(digi, "enabled");
    s_enabled = cJSON_IsTrue(en);

    const cJSON *alias_j = cJSON_GetObjectItemCaseSensitive(digi, "alias");
    if (cJSON_IsString(alias_j) && alias_j->valuestring[0]) {
        split_callssid(alias_j->valuestring, s_alias, &s_alias_ssid);
    }

    // Digipeater own callsign: falls back to aprs.callsign if not set
    const cJSON *cs_j = cJSON_GetObjectItemCaseSensitive(digi, "callsign");
    if (cJSON_IsString(cs_j) && cs_j->valuestring[0]) {
        const cJSON *ss_j = cJSON_GetObjectItemCaseSensitive(digi, "ssid");
        strncpy(s_call, cs_j->valuestring, 7); s_call[7] = '\0';
        s_call_ssid = cJSON_IsNumber(ss_j) ? (int)ss_j->valueint : 0;
    } else {
        // Inherit from aprs section
        const cJSON *aprs = cJSON_GetObjectItemCaseSensitive(cfg, "aprs");
        if (aprs) {
            const cJSON *ac = cJSON_GetObjectItemCaseSensitive(aprs, "callsign");
            const cJSON *as = cJSON_GetObjectItemCaseSensitive(aprs, "ssid");
            if (cJSON_IsString(ac)) { strncpy(s_call, ac->valuestring, 7); s_call[7] = '\0'; }
            if (cJSON_IsNumber(as)) s_call_ssid = (int)as->valueint;
        }
    }

    ESP_LOGI(TAG, "digi_init: enabled=%d alias=%s-%d via=%s-%d",
             s_enabled, s_alias, s_alias_ssid, s_call, s_call_ssid);
}

void digi_set_enabled(bool en) { s_enabled = en; }
bool digi_is_enabled(void)     { return s_enabled; }

// ─── AX.25 address helpers ───────────────────────────────────────────────────

// Read one 7-byte AX.25 address field into call (NUL-terminated, no SSID suffix).
// Returns pointer to next field. is_last set if end-of-address-field bit is set.
static const uint8_t *addr_read(const uint8_t *p,
                                 char *call_out, int *ssid_out, bool *h_out,
                                 bool *is_last_out) {
    for (int i = 0; i < 6; i++) call_out[i] = (char)(p[i] >> 1);
    call_out[6] = '\0';
    // Trim trailing spaces
    for (int i = 5; i >= 0 && call_out[i] == ' '; i--) call_out[i] = '\0';
    *ssid_out    = (p[6] >> 1) & 0x0F;
    *h_out       = (p[6] & 0x80) != 0;   // H-bit (has-been-repeated)
    *is_last_out = (p[6] & 0x01) != 0;
    return p + 7;
}

// Write one 7-byte AX.25 address field.
// last: set the end-of-address-field bit; h: set H-bit (has-been-repeated).
static void addr_write(uint8_t *p, const char *call, int ssid,
                        bool last, bool h) {
    int i;
    for (i = 0; i < 6 && call[i]; i++) p[i] = (uint8_t)((uint8_t)call[i] << 1);
    for (; i < 6; i++) p[i] = ' ' << 1;
    p[6] = (uint8_t)(((ssid & 0x0F) << 1) | (last ? 0x01 : 0x00) | (h ? 0x80 : 0x00));
    // Bits 5-7 (CRRDSSID) — set reserved bits per AX.25 v2.2
    p[6] |= 0x60;
}

// Case-insensitive strcmp for callsign matching (ignores trailing spaces).
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
    if (len < 16 || s_call[0] == '\0') return false;

    // We work on a mutable copy.
    static uint8_t frame[AX25_MAX_FRAME_LEN];
    if (len > AX25_MAX_FRAME_LEN) return false;
    memcpy(frame, buf, len);

    uint8_t *p   = frame;
    uint8_t *end = frame + len;

    // Skip DST (7 bytes)
    if (p + 7 > end) return false;
    p += 7;

    // Skip SRC (7 bytes)
    if (p + 7 > end) return false;
    bool is_last_src = (p[6] & 0x01) != 0;
    p += 7;

    // Iterate over repeater (path) fields.
    // Two passes:
    //   1) Check we haven't already digipeated this frame (our call with H=1 present).
    //   2) Find the first unused (H=0) entry matching our alias and process it.
    uint8_t *scan = p;
    bool already_repeated = false;
    bool scan_last = is_last_src;
    while (!scan_last && (scan + 7) <= end) {
        char sc[8]; int ss; bool sh, sl;
        addr_read(scan, sc, &ss, &sh, &sl);
        if (sh && call_eq(sc, s_call) && ss == s_call_ssid) {
            already_repeated = true;
            break;
        }
        scan_last = sl;
        scan += 7;
    }
    if (already_repeated) return false;

    // Find first unused path entry matching our alias
    bool did_digi = false;
    bool cur_last = is_last_src;
    while (!cur_last && (p + 7) <= end) {
        char rpt_call[8]; int rpt_ssid; bool rpt_h, rpt_last;
        addr_read(p, rpt_call, &rpt_ssid, &rpt_h, &rpt_last);

        if (!rpt_h && call_eq(rpt_call, s_alias) && rpt_ssid == s_alias_ssid) {
            // Standard WIDEn-N digipeat:
            // 1. Insert OUR callsign (H=1) immediately before this path entry.
            //    This makes the path traceable regardless of how many hops remain.
            // 2. Decrement N in the WIDEn-N entry (WIDE2-2 → WIDE2-1 → WIDE2-0).
            // 3. If N hits 0, also set H=1 on the WIDEn entry (mark as spent/trapped).
            //
            // Example — WIDE2-2 path, two hops:
            //   Before hop 1:  WIDE2-2
            //   After hop 1:   DIGI1*,WIDE2-1       (we inserted DIGI1 with H=1)
            //   After hop 2:   DIGI1*,DIGI2*,WIDE2-0* (DIGI2 inserted; WIDE2-0 spent)

            int new_ssid = rpt_ssid - 1;  // decrement N

            if (len + 7 > AX25_MAX_FRAME_LEN) {
                // No room to insert — can't trace, abort
                break;
            }

            // Make room for our 7-byte callsign entry before p
            memmove(p + 7, p, (size_t)(end - p));
            len += 7;
            end += 7;

            // Write our callsign with H=1, last=0 (the WIDEn entry follows us)
            addr_write(p, s_call, s_call_ssid, /*last=*/false, /*h=*/true);
            p += 7;  // advance past our inserted entry

            // Update the WIDEn-N entry: decrement ssid, set H only if spent (N=0)
            addr_write(p, rpt_call, new_ssid, rpt_last, /*h=*/(new_ssid == 0));

            did_digi = true;
            break;  // only digipeat once per frame
        }

        cur_last = rpt_last;
        p += 7;
    }

    if (did_digi) {
        ESP_LOGI(TAG, "Digipeating frame (len=%u) via %s-%d", (unsigned)len, s_call, s_call_ssid);
        afsk_queue_tx_frame(frame, len);
    }
    return did_digi;
}
