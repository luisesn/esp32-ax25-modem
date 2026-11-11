// Regresión del demodulador RX v2 en host (sin ESP-IDF).
//
// Incluye AFSK.cpp directamente para acceder a las funciones/variables static
// del demodulador (afsk_v2_process, adc_to_s8_v2, fir5, estado del biquad...).
// El código de producción NO se modifica: las dependencias de ESP-IDF se
// sustituyen por stubs de test/demod/stubs/.
//
// Uso:
//   demod_regress synth             batería de escenarios sintéticos con umbrales
//   demod_regress file.wav [--expect N]   decodifica un WAV (mono, 8/16 bit)
//   demod_regress corpus [dir]      todos los WAV de test/demod/corpus (o dir)
//
// La cadena por muestra replica receive_audio_task (AFSK.cpp ~1290-1350):
//   5 muestras crudas de 12 bit a 48 kHz -> FIR fir5 -> adc_to_s8_v2 (DC+AGC)
//   -> potencia de squelch -> afsk_v2_process -> rxFifo (HDLC con escapes)
//   -> decodificador de bytes equivalente a aprs_poll_v2 (CRC-CCITT).

#define CUSTOM_FRAME_SIZE 600   // igual que main/CMakeLists.txt
#include "../../main/LibAPRS-esp32-i2s/src/AFSK.cpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <algorithm>
#include "CRC-CCIT.h"

// ---- Símbolos que AFSK.cpp espera de otros módulos ---------------------------
RxStats rx_stats_v1 = {0, 0, 0, 0};
RxStats rx_stats_v2 = {0, 0, 0, 0};
extern "C" void il2p_rx_bit(uint8_t, uint8_t) {}
extern "C" void il2p_poll(void) {}
extern "C" void APRS_poll(void) {}
unsigned long custom_preamble = 350UL;
unsigned long custom_tail = 50UL;
int LibAPRS_vref = 0;

// ---- Estado de la cadena de receive_audio_task ----------------------------------
static int      g_ov_count = 0;
static int32_t  g_ov_sum   = 0;

// Decodificador de bytes equivalente a aprs_poll_v2 (LibAPRS.cpp)
struct Rx {
    bool     sync = false, escape = false;
    uint16_t crc_in = CRC_CCIT_INIT_VAL;
    size_t   len = 0;
    uint8_t  buf[AX25_MAX_FRAME_LEN];
    int      crc_ok = 0, crc_err = 0;
    std::vector<std::vector<uint8_t>> frames;   // tramas correctas (sin CRC)
};
static Rx g_rx;

// Reinicia TODO el estado del demodulador (equivale a un arranque en frío).
static void demod_reset(int squelch = 64) {
    bp_x1 = bp_x2 = bp_y1 = bp_y2 = 0;
    s_v2_disc_x0 = s_v2_disc_x1 = s_v2_disc_y0 = s_v2_disc_y1 = 0;
    s_v2_lp_x2 = s_v2_lp_y2 = 0;
    s_v2_deemph_state = 0;
    dc_offset_v2_q10 = 2048 << 10;
    agc_env_q6 = 32 << 6;
    s_v2_squelch_power = 0;
    v2_hdlc_flags_seen = 0;
    v2_was_receiving = false;
    afsk_v2_agc_peak = 0;
    afsk_v2_squelch_open = false;
    memset(v2_fir_buf, 0, sizeof(v2_fir_buf));
    s_squelch_threshold = squelch;
    s_deemphasis_enabled = false;
    s_il2p_enabled = false;
    s_rx_paused = false;
    s_cur_stats = NULL;
    memset(&rx_stats_v1, 0, sizeof(rx_stats_v1));
    memset(&rx_stats_v2, 0, sizeof(rx_stats_v2));
    // Réplica de la parte v2 de AFSK_hw_init
    memset(&afsk_v2_instance, 0, sizeof(afsk_v2_instance));
    AFSK_modem_v2 = &afsk_v2_instance;
    AFSK_modem_v2->phaseInc = MARK_INC;
    fifo_init(&AFSK_modem_v2->delayFifo, (uint8_t *)AFSK_modem_v2->delayBuf,
              sizeof(AFSK_modem_v2->delayBuf));
    fifo_init(&AFSK_modem_v2->rxFifo, AFSK_modem_v2->rxBuf, sizeof(AFSK_modem_v2->rxBuf));
    for (int i = 0; i < SAMPLESPERBIT / 2; i++) fifo_push(&AFSK_modem_v2->delayFifo, 0);
    g_ov_count = 0; g_ov_sum = 0;
    g_rx = Rx();
}

static void rx_poll(void) {
    int c;
    Rx &r = g_rx;
    while ((c = afsk_getchar_v2()) != EOF) {
        if (!r.escape && c == HDLC_FLAG) {
            if (r.len >= AX25_MIN_FRAME_LEN) {
                if (r.crc_in == AX25_CRC_CORRECT) {
                    r.crc_ok++;
                    r.frames.emplace_back(r.buf, r.buf + r.len - 2);
                } else r.crc_err++;
            }
            r.sync = true; r.crc_in = CRC_CCIT_INIT_VAL; r.len = 0;
            continue;
        }
        if (!r.escape && c == HDLC_RESET) { r.sync = false; continue; }
        if (!r.escape && c == AX25_ESC)   { r.escape = true; continue; }
        if (r.sync) {
            if (r.len < AX25_MAX_FRAME_LEN) {
                r.buf[r.len++] = (uint8_t)c;
                r.crc_in = update_crc_ccit((uint8_t)c, r.crc_in);
            } else r.sync = false;
        }
        r.escape = false;
    }
}

// Alimenta una muestra cruda de 12 bit a 48 kHz (bucle interior de receive_audio_task).
static void feed_raw(uint16_t raw) {
    v2_fir_buf[g_ov_count] = raw;
    g_ov_sum += raw;
    if (++g_ov_count >= OVERSAMPLING) {
        g_ov_sum = 0; g_ov_count = 0;
        int32_t acc = 0;
        for (int k = 0; k < OVERSAMPLING; k++) acc += (int32_t)fir5[k] * v2_fir_buf[k];
        uint16_t avg12_v2 = (uint16_t)(acc >> 8);
        int8_t s8_v2 = adc_to_s8_v2(avg12_v2);
        s_v2_squelch_power += ((int32_t)s8_v2 * s8_v2 - s_v2_squelch_power) >> 8;
        afsk_v2_squelch_open = (s_v2_squelch_power > s_squelch_threshold);
        if (afsk_v2_squelch_open) afsk_v2_process(AFSK_modem_v2, s8_v2);
        rx_poll();
    }
}

// ---- Generador sintético ----------------------------------------------------------
struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 1) {}
    uint32_t u32() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (uint32_t)(s >> 11); }
    double uni() { return (u32() + 0.5) / 4294967296.0 * 2.0; }   // (0,~2) -> usar uni01
    double uni01() { return ((u32() & 0xFFFFFF) + 0.5) / 16777216.0; }
    double gauss() { double a = uni01(), b = uni01(); return sqrt(-2 * log(a)) * cos(2 * M_PI * b); }
};

typedef std::vector<uint8_t> Bytes;

static Bytes make_ax25(const char *dst, const char *src, const char *via, const Bytes &info) {
    Bytes f;
    auto addr = [&](const char *cs, int ssid, bool last) {
        char p[7] = {' ', ' ', ' ', ' ', ' ', ' ', 0};
        for (int i = 0; i < 6 && cs[i]; i++) p[i] = cs[i];
        for (int i = 0; i < 6; i++) f.push_back((uint8_t)(p[i] << 1));
        f.push_back((uint8_t)(0x60 | (ssid << 1) | (last ? 1 : 0)));
    };
    addr(dst, 0, false);
    addr(src, 7, via == NULL);
    if (via) addr(via, 1, true);
    f.push_back(0x03); f.push_back(0xF0);
    f.insert(f.end(), info.begin(), info.end());
    return f;
}

// Trama APRS de `info_len` bytes de info, contenido pseudoaleatorio imprimible
// (pero con '}' y '~' ocasionales: dan secuencias 0x7E/0x7D en bit stuffing).
static Bytes make_frame(int idx, int info_len, Rng &rng) {
    Bytes info;
    char head[48];
    snprintf(head, sizeof head, "!4903.50N/07201.75W-T%03d ", idx);
    for (const char *p = head; *p && (int)info.size() < info_len; p++) info.push_back((uint8_t)*p);
    while ((int)info.size() < info_len) {
        uint32_t r = rng.u32() % 100;
        info.push_back(r < 3 ? '~' : r < 6 ? 0x7F : (uint8_t)(32 + rng.u32() % 95));
    }
    char src[8]; snprintf(src, sizeof src, "EA%dXYZ", idx % 10);
    return make_ax25("APRS", src, (idx & 1) ? "WIDE1" : NULL, info);
}

struct Mod {
    std::vector<uint16_t> out;   // muestras 12 bit a 48 kHz
    double phase = 0;            // fase AFSK (rad)
    double time_scale = 1.0;     // 1+deriva de reloj del transmisor (escala baudios y tonos)
    double tone_shift = 0;       // desplazamiento fijo de tonos (Hz)
    double amp = 1000;           // pico en cuentas ADC
    double sigma = 0;            // ruido gaussiano (cuentas, por muestra a 48 kHz)
    int    dc = 2048;
    bool   tone_is_space = false;
    double bit_acc = 0;          // fracción de muestra acumulada entre bits
    Rng   *rng;

    uint16_t quant(double v) {
        double x = dc + v;
        if (x < 0) x = 0;
        if (x > 4095) x = 4095;
        return (uint16_t)lrint(x);
    }
    void sample(double sig) { out.push_back(quant(sig + (sigma > 0 ? sigma * rng->gauss() : 0))); }
    void noise(double secs, double sg) {
        double old = sigma; sigma = sg;
        int n = (int)(secs * 48000);
        for (int i = 0; i < n; i++) sample(0);
        sigma = old;
    }
    // Un bit NRZI: bit 0 -> cambia de tono; bit 1 -> mantiene.
    void put_bit(int bit) {
        if (!bit) tone_is_space = !tone_is_space;
        double f = ((tone_is_space ? 2200.0 : 1200.0) + tone_shift) * time_scale;
        double dphi = 2 * M_PI * f / 48000.0;
        bit_acc += 48000.0 / (1200.0 * time_scale);
        int n = (int)bit_acc;
        bit_acc -= n;
        for (int i = 0; i < n; i++) { phase += dphi; if (phase > 2 * M_PI) phase -= 2 * M_PI; sample(amp * sin(phase)); }
    }
    void put_flag() { for (int i = 0; i < 8; i++) put_bit((0x7E >> i) & 1); }
    void put_frame(const Bytes &f, int preamble_flags, int tail_flags) {
        for (int i = 0; i < preamble_flags; i++) put_flag();
        uint16_t crc = CRC_CCIT_INIT_VAL;
        for (uint8_t b : f) crc = update_crc_ccit(b, crc);
        crc = ~crc;
        Bytes all = f; all.push_back(crc & 0xFF); all.push_back(crc >> 8);
        int ones = 0;
        for (uint8_t b : all)
            for (int i = 0; i < 8; i++) {
                int bit = (b >> i) & 1;
                put_bit(bit);
                if (bit) { if (++ones == 5) { put_bit(0); ones = 0; } } else ones = 0;
            }
        for (int i = 0; i < tail_flags; i++) put_flag();
    }
    // Bits aleatorios modulados (basura sin HDLC válido): `secs` segundos.
    void put_garbage(double secs) {
        int nb = (int)(secs * 1200);
        for (int i = 0; i < nb; i++) put_bit(rng->u32() & 1);
    }
};

struct Scenario {
    const char *name;
    int    frames;        // nº de tramas enviadas
    int    info_min, info_max;
    double amp;           // pico (cuentas ADC, 12 bit)
    double snr_db;        // SNR en banda de audio (0..4800 Hz); <=0 -> sin ruido
    double clock_dev;     // deriva de reloj del TX (fracción, p.ej. 0.01 = +1 %)
    double tone_shift;    // Hz
    int    preamble, tail;
    double gap_s;         // silencio/ruido entre tramas
    double gap_noise_x;   // sigma del ruido de hueco = x * sigma de la señal (0 = igual que la señal)
    bool   gap_silence;   // hueco sin ruido (DC puro)
    double garbage_s;     // segundos de AFSK aleatorio (sin HDLC) en cada hueco
    int    dc;            // offset de continua
    int    min_ok;        // umbral mínimo de tramas correctas (calibrado, ver main)
    int    max_false;     // máximo de tramas con CRC ok que no se enviaron (falsos positivos)
};

struct Result { int sent, ok, matched, falsepos, crc_err; };

static Result run_scenario(const Scenario &sc, uint64_t seed, bool verbose = false) {
    demod_reset();
    Rng rng(seed);
    Mod m; m.rng = &rng; m.time_scale = 1.0 + sc.clock_dev; m.tone_shift = sc.tone_shift;
    m.amp = sc.amp; m.dc = sc.dc;
    // Potencia de ruido: SNR = (A^2/2) / Pn_banda; ruido blanco a 48 kHz -> Pn_banda = sigma^2 * 4800/24000.
    double sigma = 0;
    if (sc.snr_db > 0) sigma = sqrt((sc.amp * sc.amp / 2) / pow(10, sc.snr_db / 10) * 5.0);
    m.sigma = sigma;
    double gap_sigma = sc.gap_silence ? 0 : (sc.gap_noise_x > 0 ? sc.gap_noise_x * sc.amp / 3 : sigma > 0 ? sigma : sc.amp / 6);
    std::vector<Bytes> sent;
    m.noise(0.3, gap_sigma);
    for (int i = 0; i < sc.frames; i++) {
        int len = sc.info_min + (int)(rng.u32() % (unsigned)(sc.info_max - sc.info_min + 1));
        Bytes f = make_frame(i, len, rng);
        sent.push_back(f);
        m.put_frame(f, sc.preamble, sc.tail);
        double old = m.sigma;
        m.sigma = gap_sigma;
        if (sc.garbage_s > 0) m.put_garbage(sc.garbage_s);
        m.noise(sc.gap_s, gap_sigma);
        m.sigma = old;
    }
    for (uint16_t s : m.out) feed_raw(s);
    Result r{(int)sent.size(), g_rx.crc_ok, 0, 0, g_rx.crc_err};
    std::vector<bool> used(sent.size(), false);
    for (auto &fr : g_rx.frames) {
        bool hit = false;
        for (size_t i = 0; i < sent.size(); i++)
            if (!used[i] && sent[i] == fr) { used[i] = true; hit = true; break; }
        if (hit) r.matched++; else r.falsepos++;
    }
    if (verbose) printf("  [%s seed=%llu] audio=%.1fs\n", sc.name, (unsigned long long)seed, m.out.size() / 48000.0);
    return r;
}

// ---- Salida de tramas -------------------------------------------------------------
static std::string tnc2(const Bytes &f) {
    std::string s;
    if (f.size() < 16) return "(corta)";
    auto call = [&](size_t o) {
        std::string c;
        for (int i = 0; i < 6; i++) { char ch = f[o + i] >> 1; if (ch != ' ') c += ch; }
        int ssid = (f[o + 6] >> 1) & 0xF;
        if (ssid) c += "-" + std::to_string(ssid);
        return c;
    };
    s = call(7) + ">" + call(0);
    size_t o = 14;
    bool last = f[13] & 1;
    while (!last && o + 7 <= f.size()) {
        s += "," + call(o);
        if (f[o + 6] & 0x80) s += "*";
        last = f[o + 6] & 1; o += 7;
    }
    o += 2;   // control + PID
    s += ":";
    for (; o < f.size(); o++) s += (f[o] >= 32 && f[o] < 127) ? (char)f[o] : '.';
    return s;
}

static void print_frames(void) {
    for (size_t i = 0; i < g_rx.frames.size(); i++) {
        auto &f = g_rx.frames[i];
        printf("#%zu (%zu B) ", i + 1, f.size());
        for (size_t k = 0; k < f.size() && k < 24; k++) printf("%02x", f[k]);
        if (f.size() > 24) printf("...");
        printf("\n    %s\n", tnc2(f).c_str());
    }
}

// ---- WAV ---------------------------------------------------------------------------
static bool load_wav(const char *path, std::vector<uint16_t> &raw48, int *rate_out) {
    FILE *fp = fopen(path, "rb");
    if (!fp) { fprintf(stderr, "no se puede abrir %s\n", path); return false; }
    Bytes d; uint8_t tmp[65536]; size_t n;
    while ((n = fread(tmp, 1, sizeof tmp, fp)) > 0) d.insert(d.end(), tmp, tmp + n);
    fclose(fp);
    auto rd32 = [&](size_t o) { return (uint32_t)d[o] | d[o+1] << 8 | d[o+2] << 16 | (uint32_t)d[o+3] << 24; };
    auto rd16 = [&](size_t o) { return (uint16_t)(d[o] | d[o+1] << 8); };
    if (d.size() < 44 || memcmp(&d[0], "RIFF", 4) || memcmp(&d[8], "WAVE", 4)) { fprintf(stderr, "%s: no es WAV\n", path); return false; }
    int ch = 0, bits = 0, rate = 0, fmt = 0; size_t doff = 0, dlen = 0;
    for (size_t o = 12; o + 8 <= d.size();) {
        uint32_t sz = rd32(o + 4);
        if (!memcmp(&d[o], "fmt ", 4) && o + 8 + 16 <= d.size()) { fmt = rd16(o+8); ch = rd16(o+10); rate = rd32(o+12); bits = rd16(o+22); }
        else if (!memcmp(&d[o], "data", 4)) { doff = o + 8; dlen = std::min<size_t>(sz, d.size() - doff); break; }
        o += 8 + sz + (sz & 1);
    }
    if (fmt != 1 || ch != 1 || (bits != 8 && bits != 16) || rate <= 0 || !doff) {
        fprintf(stderr, "%s: se requiere WAV PCM mono 8 o 16 bit (fmt=%d ch=%d bits=%d rate=%d)\n", path, fmt, ch, bits, rate);
        return false;
    }
    size_t ns = dlen / (bits / 8);
    std::vector<int> v(ns);
    for (size_t i = 0; i < ns; i++) {
        if (bits == 8) v[i] = ((int)d[doff + i] - 128) * 16;                       // (v-128)*16 + 2048
        else v[i] = (int)(int16_t)rd16(doff + 2 * i) / 16;                          // 16 bit -> escala 12 bit
    }
    // Remuestreo por vecino más próximo a 48 kHz (9600 Hz -> cada muestra x5, exacto).
    size_t n48 = (size_t)((double)ns * 48000.0 / rate);
    raw48.resize(n48);
    for (size_t i = 0; i < n48; i++) {
        size_t j = (size_t)((double)i * rate / 48000.0);
        if (j >= ns) j = ns - 1;
        int x = v[j] + 2048; if (x < 0) x = 0;
        if (x > 4095) x = 4095;
        raw48[i] = (uint16_t)x;
    }
    *rate_out = rate;
    return true;
}

static int read_expect(const std::string &wav) {
    std::string p = wav.substr(0, wav.size() - 4) + ".expect";
    FILE *f = fopen(p.c_str(), "r");
    if (!f) return -1;
    char line[256]; int val = -1;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        int x;
        if (sscanf(line, "min_frames=%d", &x) == 1 || sscanf(line, "%d", &x) == 1) val = x;
    }
    fclose(f);
    return val;
}

static int run_file(const char *path, int expect, bool quiet) {
    std::vector<uint16_t> raw; int rate = 0;
    if (!load_wav(path, raw, &rate)) return 2;
    demod_reset();
    // Silencio de relleno al final para vaciar HDLC/filtros.
    for (uint16_t s : raw) feed_raw(s);
    for (int i = 0; i < 48000 / 4; i++) feed_raw(2048);
    if (!quiet) print_frames();
    printf("%s: %d tramas (CRC err: %d, flags: %u, %.1f s @ %d Hz)", path, g_rx.crc_ok, g_rx.crc_err,
           (unsigned)rx_stats_v2.hdlc_flags, raw.size() / 48000.0, rate);
    if (expect >= 0) {
        bool ok = g_rx.crc_ok >= expect;
        printf("  esperado >= %d: %s", expect, ok ? "OK" : "FALLO");
        printf("\n");
        return ok ? 0 : 1;
    }
    printf("\n");
    return 0;
}

static int run_corpus(const char *dir) {
    DIR *d = opendir(dir);
    if (!d) { fprintf(stderr, "sin directorio de corpus: %s\n", dir); return 0; }
    std::vector<std::string> names;
    while (dirent *e = readdir(d)) {
        std::string n = e->d_name;
        if (n.size() > 4 && n.substr(n.size() - 4) == ".wav") names.push_back(n);
    }
    closedir(d);
    std::sort(names.begin(), names.end());
    int fails = 0;
    for (auto &n : names) {
        std::string p = std::string(dir) + "/" + n;
        fails += run_file(p.c_str(), read_expect(p), true) != 0;
    }
    printf("corpus: %zu ficheros, %d fallos\n", names.size(), fails);
    return fails ? 1 : 0;
}

// ---- Batería sintética -----------------------------------------------------------------
// Umbrales (min_ok) calibrados con el demodulador v2 de v1.11.0 sumando 2 semillas
// (2 x frames tramas por escenario); se han fijado ~10 % por debajo de lo medido
// (redondeando hacia abajo) para tolerar cambios menores de filtros sin ser frágiles.
// Comportamiento determinista (PRNG propio): una regresión real baja el contador.
// Campos: nombre, tramas, info_min, info_max, amp, snr, clock, shift, pre, tail, gap,
//         gap_noise_x, gap_silence, garbage_s, dc, min_ok (suma de 2 semillas), max_false
static const Scenario kScenarios[] = {
#include "scenarios.inc"
};

static int run_synth(bool calibrate) {
    const int seeds = 2;
    int fails = 0;
    printf("%-26s %6s %6s %6s %6s %7s %6s\n", "escenario", "env", "ok", "match", "falsos", "crcerr", "umbral");
    for (const Scenario &sc : kScenarios) {
        Result t{0, 0, 0, 0, 0};
        for (int s = 1; s <= seeds; s++) {
            Result r = run_scenario(sc, (uint64_t)s * 1000 + 7);
            t.sent += r.sent; t.ok += r.ok; t.matched += r.matched; t.falsepos += r.falsepos; t.crc_err += r.crc_err;
        }
        bool pass = t.matched >= sc.min_ok && t.falsepos <= sc.max_false;
        if (calibrate) printf("%-26s %6d %6d %6d %6d %7d   sugerido %d\n", sc.name, t.sent, t.ok, t.matched, t.falsepos, t.crc_err, (int)floor(t.matched * 0.9));
        else printf("%-26s %6d %6d %6d %6d %7d %6d  %s\n", sc.name, t.sent, t.ok, t.matched, t.falsepos, t.crc_err, sc.min_ok, pass ? "OK" : "FALLO");
        if (!pass && !calibrate) fails++;
    }
    printf("%s (%d escenarios fallidos)\n", fails ? "FALLO" : "TODO OK", fails);
    return fails ? 1 : 0;
}


// Genera un WAV 8 bit 9600 Hz (formato de /api/rx/capture) con tramas sintéticas.
// Sirve para probar el camino de ficheros y como corpus mínimo de ejemplo.
static int gen_wav(const char *path, int nframes, double snr_db, uint64_t seed) {
    demod_reset();
    Rng rng(seed);
    Mod m; m.rng = &rng; m.amp = 1000;
    double sigma = sqrt((m.amp * m.amp / 2) / pow(10, snr_db / 10) * 5.0);
    m.sigma = sigma;
    m.noise(0.5, sigma);
    for (int i = 0; i < nframes; i++) {
        m.put_frame(make_frame(i, 40 + (i * 23) % 120, rng), 30, 3);
        m.noise(0.6, sigma);
    }
    size_t n = m.out.size() / 5;
    Bytes pcm(n);
    for (size_t i = 0; i < n; i++) {
        int acc = 0;
        for (int k = 0; k < 5; k++) acc += m.out[i * 5 + k];
        int v = (acc / 5 - 2048) / 16 + 128;
        pcm[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
    }
    FILE *f = fopen(path, "wb");
    if (!f) return 2;
    uint8_t h[44]; memcpy(h, "RIFF\0\0\0\0WAVEfmt \x10\0\0\0\x01\0\x01\0", 24);
    uint32_t rate = 9600, br = 9600, dl = (uint32_t)n, rl = 36 + dl;
    memcpy(h + 4, &rl, 4);
    memcpy(h + 24, &rate, 4); memcpy(h + 28, &br, 4);
    h[32] = 1; h[33] = 0; h[34] = 8; h[35] = 0;
    memcpy(h + 36, "data", 4); memcpy(h + 40, &dl, 4);
    fwrite(h, 1, 44, f); fwrite(pcm.data(), 1, n, f); fclose(f);
    printf("%s: %d tramas, %zu muestras\n", path, nframes, n);
    return 0;
}

#ifndef CORPUS_DIR
#define CORPUS_DIR "corpus"
#endif

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "uso: %s synth | calibrate | genwav out.wav [n] [snr] | corpus [dir] | file.wav [--expect N]\n", argv[0]); return 2; }
    std::string a = argv[1];
    if (a == "synth") return run_synth(false);
    if (a == "calibrate") return run_synth(true);
    if (a == "genwav" && argc >= 3) return gen_wav(argv[2], argc > 3 ? atoi(argv[3]) : 8, argc > 4 ? atof(argv[4]) : 15, 42);
    if (a == "corpus") return run_corpus(argc > 2 ? argv[2] : CORPUS_DIR);
    int expect = -1;
    for (int i = 2; i + 1 < argc; i++) if (!strcmp(argv[i], "--expect")) expect = atoi(argv[i + 1]);
    if (expect < 0) expect = read_expect(a);
    return run_file(argv[1], expect, false);
}
