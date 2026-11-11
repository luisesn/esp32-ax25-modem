#include "kiss.h"
#include "transport.h"
#include "LibAPRS-esp32-i2s/src/AX25.h"  // AX25_MAX_FRAME_LEN
#include "LibAPRS-esp32-i2s/src/AFSK.h"  // afsk_csma_*
#include "esp_log.h"
#include <stdbool.h>
#include <stdlib.h>

static const char *TAG = "kiss";

// KISS encoded worst case: FEND + CMD + 2*frame + FEND
#define KISS_MAX_ENCODED (2 + 1 + AX25_MAX_FRAME_LEN * 2)

typedef enum {
    S_IDLE,
    S_CMD,
    S_DATA,
    S_ESC,
} kiss_state_t;

// Estado del decodificador por cliente (F5): cada conexión KISS tiene el suyo.
struct kiss_ctx {
    kiss_state_t state;
    uint8_t      buf[AX25_MAX_FRAME_LEN];
    size_t       len;
    uint8_t      cmd;
    // Set when the host frame exceeds buf. The frame is then discarded at its
    // closing FEND instead of being transmitted truncated: a truncated AX.25 frame
    // goes on air with a valid CRC, so the peer accepts it and silently drops the
    // mangled payload higher up — which looks like an unexplained TX stall.
    bool         overflow;
};

static kiss_frame_cb s_on_frame;
static kiss_ctx_t   *s_default_ctx;   // contexto de kiss_rx_byte()/kiss_reset() (compatibilidad)

kiss_ctx_t *kiss_ctx_new(void) {
    kiss_ctx_t *c = (kiss_ctx_t *)calloc(1, sizeof(*c));
    if (c) c->state = S_IDLE;
    return c;
}

void kiss_ctx_free(kiss_ctx_t *c) { free(c); }

void kiss_ctx_reset(kiss_ctx_t *c) {
    c->state    = S_IDLE;
    c->len      = 0;
    c->overflow = false;
}

void kiss_init(kiss_frame_cb on_frame) {
    s_on_frame = on_frame;
    if (!s_default_ctx) s_default_ctx = kiss_ctx_new();
    if (s_default_ctx) kiss_ctx_reset(s_default_ctx);
}

void kiss_reset(void) {
    if (s_default_ctx) kiss_ctx_reset(s_default_ctx);
}

void kiss_rx_byte(uint8_t b) {
    if (s_default_ctx) kiss_ctx_rx_byte(s_default_ctx, b);
}

// Append one decoded payload byte, flagging overflow instead of truncating.
static void kiss_put(kiss_ctx_t *c, uint8_t b) {
    if (c->len < AX25_MAX_FRAME_LEN) c->buf[c->len++] = b;
    else                             c->overflow = true;
}

void kiss_ctx_rx_byte(kiss_ctx_t *c, uint8_t b) {
    switch (c->state) {
    case S_IDLE:
        if (b == KISS_FEND) { c->state = S_CMD; c->len = 0; c->overflow = false; }
        break;

    case S_CMD:
        if (b == KISS_FEND) { c->len = 0; c->overflow = false; break; }  // doble FEND: ignorar
        c->cmd   = b;
        c->state = S_DATA;
        break;

    case S_DATA:
        if (b == KISS_FEND) {
            if (c->overflow) {
                ESP_LOGW(TAG, "trama del host > %d B: descartada (reduce la MTU del host)",
                         AX25_MAX_FRAME_LEN);
            } else if ((c->cmd & 0x0F) == 0x00 && c->len > 0 && s_on_frame) {
                // Solo entregar tramas DATA (cmd bajo nibble = 0x0, puerto 0)
                s_on_frame(c->buf, c->len);
            }
            else if (c->len == 1 && !c->overflow) {
                // Parámetros KISS que afectan al acceso al canal (sólo con LBT activo).
                // TXDELAY/TXTAIL/FullDuplex se ignoran: el preámbulo sale de aprs.* en config.json.
                uint8_t persist; uint32_t slot_ms;
                afsk_csma_get(&persist, &slot_ms);
                switch (c->cmd & 0x0F) {
                case 0x02: persist = c->buf[0]; afsk_csma_configure(persist, slot_ms);
                           ESP_LOGI(TAG, "KISS P=%u", (unsigned)persist); break;
                case 0x03: slot_ms = (uint32_t)c->buf[0] * 10u; afsk_csma_configure(persist, slot_ms);
                           ESP_LOGI(TAG, "KISS SlotTime=%lu ms", (unsigned long)slot_ms); break;
                default: break;
                }
            }
            c->state    = S_CMD;
            c->len      = 0;
            c->overflow = false;
        } else if (b == KISS_FESC) {
            c->state = S_ESC;
        } else {
            kiss_put(c, b);
        }
        break;

    case S_ESC:
        c->state = S_DATA;
        if (b == KISS_FEND) { kiss_ctx_rx_byte(c, b); break; }  // FEND tras FESC: cierra la trama (B23)
        if      (b == KISS_TFEND) kiss_put(c, KISS_FEND);
        else if (b == KISS_TFESC) kiss_put(c, KISS_FESC);
        // cualquier otro byte tras escape: descartar (error de protocolo)
        break;
    }
}

void kiss_send_frame(const uint8_t *frame, size_t len) {
    static uint8_t out[KISS_MAX_ENCODED];
    size_t i = 0;

    out[i++] = KISS_FEND;
    out[i++] = 0x00;  // port 0, comando DATA

    for (size_t j = 0; j < len; j++) {
        uint8_t b = frame[j];
        if (b == KISS_FEND) {
            out[i++] = KISS_FESC;
            out[i++] = KISS_TFEND;
        } else if (b == KISS_FESC) {
            out[i++] = KISS_FESC;
            out[i++] = KISS_TFESC;
        } else {
            out[i++] = b;
        }
    }

    out[i++] = KISS_FEND;
    transport_write(out, i);
}
