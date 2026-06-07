/*
 * Reed-Solomon codec for IL2P
 * Adapted from Phil Karn KA9Q's libfec (LGPL) and Direwolf/WB2OSZ.
 *
 * Two independent instantiations:
 *   GF(2^8), poly=0x11D, fcr=1, nroots=16  → RS(255,239)
 *   GF(2^4), poly=0x13,  fcr=1, nroots=2   → RS(15,13)
 */

#include "rs_codec.h"
#include <string.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Generic GF(2^m) Reed-Solomon building blocks
 * ═══════════════════════════════════════════════════════════════════════════ */

/* GF(2^8) ------------------------------------------------------------------ */
#define GF8_POLY   0x11D   /* x^8 + x^4 + x^3 + x^2 + 1                     */
#define RS8_N      255
#define RS8_NROOTS 16
#define RS8_K      239     /* RS8_N - RS8_NROOTS                               */
#define RS8_FCR    1       /* first consecutive root = α^1                     */

static uint8_t  gf8_exp[512];          /* α^i, doubled for wrap-around         */
static uint8_t  gf8_log[256];          /* log_α(x), gf8_log[0] undefined       */
static uint8_t  rs8_gp[RS8_NROOTS];   /* generator poly coefficients (log)     */
static bool     gf8_ready = false;

static void gf8_init(void)
{
    uint16_t sr = 1;
    for (int i = 0; i < 255; i++) {
        gf8_exp[i] = (uint8_t)sr;
        gf8_log[(uint8_t)sr] = (uint8_t)i;
        sr <<= 1;
        if (sr & 0x100) sr ^= GF8_POLY;
    }
    for (int i = 255; i < 512; i++) gf8_exp[i] = gf8_exp[i - 255];

    /* Build generator polynomial g(x) = ∏(x − α^(FCR+i)) for i=0..NROOTS-1
     * Store coefficients g[0..NROOTS-1] in log form (leading coeff = 1 omitted).
     * After multiplication, gp[j] = log( coefficient of x^j ) with gp[NROOTS]=1.
     * We use a temporary full array then convert to log. */
    uint8_t tmp[RS8_NROOTS + 1];
    tmp[0] = 1;
    for (int j = 1; j <= RS8_NROOTS; j++) tmp[j] = 0;

    for (int i = 0; i < RS8_NROOTS; i++) {
        uint8_t root = gf8_exp[RS8_FCR + i];
        for (int j = i + 1; j > 0; j--) {
            if (tmp[j - 1] != 0)
                tmp[j] ^= gf8_exp[(gf8_log[tmp[j - 1]] + gf8_log[root]) % 255];
            /* else: 0 * root = 0, no change to tmp[j] from this term */
        }
        tmp[0] = gf8_exp[(gf8_log[tmp[0]] + gf8_log[root]) % 255];
    }
    /* tmp[0..RS8_NROOTS]: coefficients, tmp[RS8_NROOTS] should be 1 */
    for (int j = 0; j < RS8_NROOTS; j++)
        rs8_gp[j] = (tmp[j] != 0) ? gf8_log[tmp[j]] : 255; /* 255 = log(0) sentinel */

    gf8_ready = true;
}

static inline uint8_t gf8_mul(uint8_t a, uint8_t b)
{
    if (a == 0 || b == 0) return 0;
    return gf8_exp[(int)gf8_log[a] + (int)gf8_log[b]];
}

static inline uint8_t gf8_inv(uint8_t a)
{
    return gf8_exp[255 - (int)gf8_log[a]];
}

/* GF(2^4) ------------------------------------------------------------------ */
#define GF4_POLY   0x13    /* x^4 + x + 1                                      */
#define RS4_N      15
#define RS4_NROOTS 2
#define RS4_K      13
#define RS4_FCR    1

static uint8_t  gf4_exp[32];
static uint8_t  gf4_log[16];
static uint8_t  rs4_gp[RS4_NROOTS];   /* coefficients in log form              */
static bool     gf4_ready = false;

static void gf4_init(void)
{
    uint8_t sr = 1;
    for (int i = 0; i < 15; i++) {
        gf4_exp[i] = sr;
        gf4_log[sr] = (uint8_t)i;
        sr <<= 1;
        if (sr & 0x10) sr ^= GF4_POLY;
    }
    for (int i = 15; i < 32; i++) gf4_exp[i] = gf4_exp[i - 15];

    /* g(x) = (x − α^1)(x − α^2) = x^2 + (α^1 + α^2)x + α^3
     * α^1 = 2, α^2 = 4, α^3 = 8  in GF(2^4) with poly 0x13 */
    uint8_t tmp[RS4_NROOTS + 1];
    tmp[0] = 1;
    for (int j = 1; j <= RS4_NROOTS; j++) tmp[j] = 0;

    for (int i = 0; i < RS4_NROOTS; i++) {
        uint8_t root = gf4_exp[RS4_FCR + i];
        for (int j = i + 1; j > 0; j--) {
            if (tmp[j - 1] != 0)
                tmp[j] ^= gf4_exp[(gf4_log[tmp[j - 1]] + gf4_log[root]) % 15];
        }
        tmp[0] = gf4_exp[(gf4_log[tmp[0]] + gf4_log[root]) % 15];
    }
    for (int j = 0; j < RS4_NROOTS; j++)
        rs4_gp[j] = (tmp[j] != 0) ? gf4_log[tmp[j]] : 255;

    gf4_ready = true;
}

static inline uint8_t gf4_mul(uint8_t a, uint8_t b)
{
    if (a == 0 || b == 0) return 0;
    return gf4_exp[(int)gf4_log[a] + (int)gf4_log[b]];
}

static inline uint8_t gf4_inv(uint8_t a)
{
    return gf4_exp[15 - (int)gf4_log[a]];
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Generic encode/decode macros (avoids code duplication for GF8/GF4)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * We use an X-macro pattern: define the two field variants via #include of
 * the same algorithm, parameterised by macros. */

/* ── RS(255,239) encode ──────────────────────────────────────────────────── */
void rs8_encode(const uint8_t *data, int dlen, uint8_t *parity)
{
    if (!gf8_ready) gf8_init();

    memset(parity, 0, RS8_NROOTS);

    for (int i = 0; i < dlen; i++) {
        uint8_t feedback = data[i] ^ parity[0];
        if (feedback != 0) {
            int fl = gf8_log[feedback];
            for (int j = 1; j < RS8_NROOTS; j++) {
                if (rs8_gp[j] != 255)
                    parity[j] ^= gf8_exp[fl + rs8_gp[j]];
            }
            memmove(parity, parity + 1, RS8_NROOTS - 1);
            parity[RS8_NROOTS - 1] = (rs8_gp[0] != 255) ? gf8_exp[fl + rs8_gp[0]] : 0;
        } else {
            memmove(parity, parity + 1, RS8_NROOTS - 1);
            parity[RS8_NROOTS - 1] = 0;
        }
    }
}

/* ── RS(255,239) decode (Berlekamp-Massey + Chien + Forney) ─────────────── */
int rs8_decode(uint8_t *block, int dlen)
{
    if (!gf8_ready) gf8_init();

    int pad = RS8_K - dlen;   /* shortening */
    int nerr = 0;

    /* 1. Syndrome computation */
    uint8_t s[RS8_NROOTS];
    for (int i = 0; i < RS8_NROOTS; i++) {
        uint8_t syn = 0;
        for (int j = 0; j < dlen + RS8_NROOTS; j++)
            syn = gf8_mul(syn, gf8_exp[RS8_FCR + i]) ^ block[j];
        s[i] = syn;
    }

    /* Check if all syndromes zero */
    bool all_zero = true;
    for (int i = 0; i < RS8_NROOTS; i++) if (s[i]) { all_zero = false; break; }
    if (all_zero) return 0;

    /* 2. Berlekamp-Massey: find error locator polynomial σ(x) */
    uint8_t lambda[RS8_NROOTS + 1]; memset(lambda, 0, sizeof(lambda)); lambda[0] = 1;
    uint8_t b[RS8_NROOTS + 1];      memset(b,      0, sizeof(b));      b[0] = 1;
    int L = 0;

    for (int r = 0; r < RS8_NROOTS; r++) {
        /* Discrepancy */
        uint8_t delta = s[r];
        for (int i = 1; i <= L; i++)
            delta ^= gf8_mul(lambda[i], s[r - i]);

        /* Shift b */
        memmove(b + 1, b, RS8_NROOTS * sizeof(uint8_t));
        b[0] = 0;

        if (delta != 0) {
            uint8_t t[RS8_NROOTS + 1];
            for (int i = 0; i <= RS8_NROOTS; i++)
                t[i] = lambda[i] ^ gf8_mul(delta, b[i]);

            if (2 * L <= r) {
                uint8_t di = gf8_inv(delta);
                for (int i = 0; i <= RS8_NROOTS; i++)
                    b[i] = gf8_mul(lambda[i], di);
                L = r + 1 - L;
            }
            memcpy(lambda, t, sizeof(lambda));
        }
    }

    if (L > RS8_NROOTS / 2) return -1; /* too many errors */

    /* 3. Chien search: find roots of σ(x) */
    int pos[RS8_NROOTS / 2];
    int nroots_found = 0;
    int block_len = dlen + RS8_NROOTS;

    for (int i = 0; i < RS8_N; i++) {
        /* Evaluate σ(α^(-i)) = σ(α^(255-i)) */
        uint8_t val = 1;
        for (int j = 1; j <= L; j++)
            val ^= gf8_mul(lambda[j], gf8_exp[(j * (255 - i)) % 255]);
        if (val == 0) {
            /* Error at position n-1-i in the full (non-shortened) codeword */
            int p = RS8_N - 1 - i - pad;
            if (p >= 0 && p < block_len) {
                pos[nroots_found++] = p;
                if (nroots_found > L) break;
            }
        }
    }

    if (nroots_found != L) return -1;

    /* 4. Compute error evaluator Ω(x) = S(x)*σ(x) mod x^{2t}.
     * Ω_i = Σ_{m=0}^{i} s[m] * λ[i-m]  for i=0..2t-1 */
    uint8_t omega[RS8_NROOTS];
    for (int i = 0; i < RS8_NROOTS; i++) {
        uint8_t v = 0;
        for (int m = 0; m <= i && m <= L; m++)
            v ^= gf8_mul(s[m], lambda[i - m]);
        omega[i] = v;
    }

    /* 5. Forney: e_k = X_k^{FCR} * Ω(X_k^{-1}) / σ'(X_k^{-1})
     * X_k = α^{xi_log},  X_k^{-1} = α^{255-xi_log}
     * σ'(x) = Σ_{j odd} λ_j * x^{j-1}  (formal derivative in GF(2)) */
    for (int k = 0; k < nroots_found; k++) {
        int xi_log     = RS8_N - 1 - pos[k] - pad;   /* log(X_k)     */
        int xi_inv_log = (255 - xi_log % 255) % 255;  /* log(X_k^{-1}) */

        /* σ'(X_k^{-1}) = Σ_{j=1,3,...} λ_j * (X_k^{-1})^{j-1} */
        uint8_t sigma_prime = 0;
        for (int j = 1; j <= L; j += 2) {
            if (lambda[j] == 0) continue;
            int exp = (int)(((uint32_t)(j - 1) * (uint32_t)xi_inv_log) % 255);
            sigma_prime ^= gf8_mul(lambda[j], (exp == 0) ? 1 : gf8_exp[exp]);
        }

        /* Ω(X_k^{-1}) via Horner on omega[0..2t-1] */
        uint8_t xi_inv  = (xi_inv_log == 0) ? 1 : gf8_exp[xi_inv_log];
        uint8_t omega_val = 0;
        for (int j = RS8_NROOTS - 1; j >= 0; j--)
            omega_val = gf8_mul(omega_val, xi_inv) ^ omega[j];

        if (sigma_prime == 0) return -1;
        /* X_k^{FCR} = α^{xi_log * FCR} */
        int xk_fcr_log = (int)(((uint32_t)xi_log % 255) * RS8_FCR % 255);
        uint8_t xk_fcr = (xk_fcr_log == 0 && xi_log == 0) ? 1 : gf8_exp[xk_fcr_log];
        uint8_t err = gf8_mul(xk_fcr, gf8_mul(omega_val, gf8_inv(sigma_prime)));
        if (err != 0) {
            block[pos[k]] ^= err;
            nerr++;
        }
    }

    /* Verify */
    for (int i = 0; i < RS8_NROOTS; i++) {
        uint8_t syn = 0;
        for (int j = 0; j < dlen + RS8_NROOTS; j++)
            syn = gf8_mul(syn, gf8_exp[RS8_FCR + i]) ^ block[j];
        if (syn) return -1;
    }

    return nerr;
}

/* ── RS(15,13) encode ────────────────────────────────────────────────────── */
void rs4_encode(const uint8_t *data, uint8_t *parity)
{
    if (!gf4_ready) gf4_init();

    memset(parity, 0, RS4_NROOTS);

    for (int i = 0; i < RS4_K; i++) {
        uint8_t feedback = data[i] ^ parity[0];
        if (feedback != 0) {
            int fl = gf4_log[feedback];
            for (int j = 1; j < RS4_NROOTS; j++) {
                if (rs4_gp[j] != 255)
                    parity[j] ^= gf4_exp[fl + rs4_gp[j]];
            }
            memmove(parity, parity + 1, RS4_NROOTS - 1);
            parity[RS4_NROOTS - 1] = (rs4_gp[0] != 255) ? gf4_exp[fl + rs4_gp[0]] : 0;
        } else {
            memmove(parity, parity + 1, RS4_NROOTS - 1);
            parity[RS4_NROOTS - 1] = 0;
        }
    }
}

/* ── RS(15,13) decode ────────────────────────────────────────────────────── */
int rs4_decode(uint8_t *block)
{
    if (!gf4_ready) gf4_init();

    /* Syndromes s[0], s[1] for roots α^1, α^2 */
    uint8_t s[RS4_NROOTS];
    for (int i = 0; i < RS4_NROOTS; i++) {
        uint8_t syn = 0;
        for (int j = 0; j < RS4_N; j++)
            syn = gf4_mul(syn, gf4_exp[RS4_FCR + i]) ^ block[j];
        s[i] = syn;
    }

    if (s[0] == 0 && s[1] == 0) return 0;

    /* For t=1 (one error), the error locator is σ(x) = 1 + σ_1 * x.
     * σ_1 = s[0], and the root gives the position.
     * Verify: s[1] = s[0]^2 (for t=1 with FCR=1 in GF(2^4)).  */
    if (gf4_mul(s[0], s[0]) != s[1]) return -1; /* more than 1 error */

    /* Find position: α^(-(pos)) = σ_1 → pos = -log(s[0]) mod 15 */
    if (s[0] == 0) return -1;
    int err_pos = (15 - gf4_log[s[0]]) % 15;
    if (err_pos >= RS4_N) return -1;

    /* Error magnitude: e = s[0] / α^(FCR * err_pos) = s[0] * α^(-err_pos) */
    int xi_log = (15 - err_pos) % 15; /* log(α^(-err_pos)) */
    uint8_t err_val = gf4_mul(s[0], gf4_exp[(uint32_t)xi_log * RS4_FCR % 15]);

    if (err_val == 0) return -1;
    block[err_pos] ^= err_val;
    return 1;
}
