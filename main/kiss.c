#include "kiss.h"
#include "transport.h"
#include "LibAPRS-esp32-i2s/src/AX25.h"  // AX25_MAX_FRAME_LEN

// KISS encoded worst case: FEND + CMD + 2*frame + FEND
#define KISS_MAX_ENCODED (2 + 1 + AX25_MAX_FRAME_LEN * 2)

typedef enum {
    S_IDLE,
    S_CMD,
    S_DATA,
    S_ESC,
} kiss_state_t;

static kiss_frame_cb s_on_frame;
static kiss_state_t  s_state = S_IDLE;
static uint8_t       s_buf[AX25_MAX_FRAME_LEN];
static size_t        s_len;
static uint8_t       s_cmd;

void kiss_init(kiss_frame_cb on_frame) {
    s_on_frame = on_frame;
    s_state    = S_IDLE;
    s_len      = 0;
}

void kiss_reset(void) {
    s_state = S_IDLE;
    s_len   = 0;
}

void kiss_rx_byte(uint8_t b) {
    switch (s_state) {
    case S_IDLE:
        if (b == KISS_FEND) { s_state = S_CMD; s_len = 0; }
        break;

    case S_CMD:
        if (b == KISS_FEND) { s_len = 0; break; }  // doble FEND: ignorar
        s_cmd   = b;
        s_state = S_DATA;
        break;

    case S_DATA:
        if (b == KISS_FEND) {
            // Solo entregar tramas DATA (cmd bajo nibble = 0x0, puerto 0)
            if ((s_cmd & 0x0F) == 0x00 && s_len > 0 && s_on_frame)
                s_on_frame(s_buf, s_len);
            s_state = S_CMD;
            s_len   = 0;
        } else if (b == KISS_FESC) {
            s_state = S_ESC;
        } else if (s_len < AX25_MAX_FRAME_LEN) {
            s_buf[s_len++] = b;
        }
        break;

    case S_ESC:
        s_state = S_DATA;
        if      (b == KISS_TFEND && s_len < AX25_MAX_FRAME_LEN) s_buf[s_len++] = KISS_FEND;
        else if (b == KISS_TFESC && s_len < AX25_MAX_FRAME_LEN) s_buf[s_len++] = KISS_FESC;
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
