// Códec IMA ADPCM (bloques de 512 B / 1017 muestras). Declaraciones en audio_stream.h.
#include "audio_stream.h"
#include <stdint.h>

// ─── IMA ADPCM ───────────────────────────────────────────────────────────────

static const int step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41,
    45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190,
    209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499,
    2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845,
    8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385,
    24623, 27086, 29794, 32767
};
static const int index_table[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };

static uint8_t adpcm_encode(adpcm_state_t *st, int16_t sample) {
    int diff = sample - (int16_t)st->predictor;
    uint8_t nibble = 0;
    if (diff < 0) { nibble = 8; diff = -diff; }
    int step = step_table[(int)st->step_index];
    if (diff >= step)       { nibble |= 4; diff -= step; }
    if (diff >= (step >> 1)){ nibble |= 2; diff -= step >> 1; }
    if (diff >= (step >> 2)){ nibble |= 1; }

    int delta = step >> 3;
    if (nibble & 4) delta += step;
    if (nibble & 2) delta += step >> 1;
    if (nibble & 1) delta += step >> 2;
    if (nibble & 8) st->predictor -= delta;
    else            st->predictor += delta;
    if (st->predictor >  32767) st->predictor =  32767;
    if (st->predictor < -32768) st->predictor = -32768;

    st->step_index += index_table[nibble & 7];
    if (st->step_index < 0)  st->step_index = 0;
    if (st->step_index > 88) st->step_index = 88;
    return nibble & 0x0F;
}

// Codifica 1017 muestras int8 en un bloque ADPCM WAV de 512 bytes.
// samples[0] se guarda como predictor de cabecera (no encoded); samples[1..1016]
// se empacan como nibbles (low nibble primero), 2 por byte → 508 bytes de datos.
void encode_block(adpcm_state_t *st, const int8_t *samples, uint8_t *block) {
    // La primera muestra es el predictor inicial (no se codifica como nibble)
    int16_t first = (int16_t)samples[0] << 8;
    st->predictor  = first;
    // step_index se hereda del bloque anterior (estado continuo)

    block[0] = (uint8_t)(st->predictor & 0xFF);
    block[1] = (uint8_t)((st->predictor >> 8) & 0xFF);
    block[2] = (uint8_t)st->step_index;
    block[3] = 0;

    for (int i = 0; i < 508; i++) {
        int si = 1 + i * 2;
        uint8_t lo = adpcm_encode(st, (int16_t)samples[si]     << 8);
        uint8_t hi = adpcm_encode(st, (int16_t)samples[si + 1] << 8);
        block[4 + i] = (uint8_t)(lo | (hi << 4));
    }
}

static int8_t adpcm_decode_nibble(adpcm_state_t *st, uint8_t nibble) {
    int step  = step_table[(int)st->step_index];
    int delta = step >> 3;
    if (nibble & 4) delta += step;
    if (nibble & 2) delta += step >> 1;
    if (nibble & 1) delta += step >> 2;
    if (nibble & 8) st->predictor -= delta;
    else            st->predictor += delta;
    if (st->predictor >  32767) st->predictor =  32767;
    if (st->predictor < -32768) st->predictor = -32768;
    st->step_index += index_table[nibble & 7];
    if (st->step_index < 0)  st->step_index = 0;
    if (st->step_index > 88) st->step_index = 88;
    return (int8_t)(st->predictor >> 8);
}

void decode_block(const uint8_t *block, int8_t *samples) {
    adpcm_state_t st;
    st.predictor  = (int16_t)((uint16_t)block[0] | ((uint16_t)block[1] << 8));
    st.step_index = (int8_t)block[2];
    if (st.step_index < 0)  st.step_index = 0;
    if (st.step_index > 88) st.step_index = 88;
    samples[0] = (int8_t)(st.predictor >> 8);
    for (int i = 0; i < 508; i++) {
        uint8_t byte = block[4 + i];
        samples[1 + i * 2]     = adpcm_decode_nibble(&st, byte & 0x0F);
        samples[1 + i * 2 + 1] = adpcm_decode_nibble(&st, byte >> 4);
    }
}
