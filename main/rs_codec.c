/*
 * Reed-Solomon para IL2P: GF(2^8), poly 0x11D, fcr=0, prim=1 (igual que
 * Dire Wolf / libfec con esos parámetros). Genérico en nroots (≤16).
 * Decodificación: síndromes + Berlekamp-Massey + Chien + Forney, con
 * verificación final de síndromes.
 */

#include "rs_codec.h"
#include <string.h>
#include <stdbool.h>

#define GF_POLY 0x11D
#define MAXR    16

static uint8_t gexp[512];
static uint8_t glog[256];
static bool    gready = false;

static void gf_init(void)
{
    unsigned sr = 1;
    for (int i = 0; i < 255; i++) {
        gexp[i] = (uint8_t)sr;
        glog[sr] = (uint8_t)i;
        sr <<= 1;
        if (sr & 0x100) sr ^= GF_POLY;
    }
    for (int i = 255; i < 512; i++) gexp[i] = gexp[i - 255];
    gready = true;
}

static inline uint8_t gmul(uint8_t a, uint8_t b)
{
    return (a && b) ? gexp[glog[a] + glog[b]] : 0;
}

static inline uint8_t gdiv(uint8_t a, uint8_t b)   /* b != 0 */
{
    return a ? gexp[glog[a] + 255 - glog[b]] : 0;
}

/* g(x) = ∏_{i=0}^{n-1} (x + α^i); g[0] = 1 (coef. de x^n), g[k] = coef. de x^(n-k). */
static void gen_poly(int n, uint8_t *g)
{
    memset(g, 0, MAXR + 1);
    g[0] = 1;
    for (int i = 0; i < n; i++) {
        uint8_t r = gexp[i];
        for (int k = i + 1; k >= 1; k--)
            g[k] ^= gmul(g[k - 1], r);
    }
}

void il2p_rs_encode(const uint8_t *data, int dlen, int nroots, uint8_t *parity)
{
    if (!gready) gf_init();
    uint8_t g[MAXR + 1];
    gen_poly(nroots, g);
    memset(parity, 0, (size_t)nroots);
    for (int i = 0; i < dlen; i++) {
        uint8_t fb = data[i] ^ parity[0];
        for (int j = 0; j < nroots - 1; j++)
            parity[j] = parity[j + 1] ^ gmul(fb, g[j + 1]);
        parity[nroots - 1] = gmul(fb, g[nroots]);
    }
}

static void syndromes(const uint8_t *blk, int blen, int n, uint8_t *s)
{
    for (int i = 0; i < n; i++) {
        uint8_t acc = 0, a = gexp[i];
        for (int j = 0; j < blen; j++) acc = gmul(acc, a) ^ blk[j];
        s[i] = acc;
    }
}

int il2p_rs_decode(uint8_t *block, int dlen, int nroots)
{
    if (!gready) gf_init();
    const int n = nroots, blen = dlen + nroots;
    if (blen > 255 || n > MAXR) return -1;

    uint8_t s[MAXR];
    syndromes(block, blen, n, s);
    bool clean = true;
    for (int i = 0; i < n; i++) if (s[i]) { clean = false; break; }
    if (clean) return 0;

    /* Berlekamp-Massey */
    uint8_t lam[MAXR + 1] = {1}, b[MAXR + 1] = {1}, t[MAXR + 1];
    int L = 0, m = 1;
    uint8_t bd = 1;
    for (int r = 0; r < n; r++) {
        uint8_t d = s[r];
        for (int i = 1; i <= L; i++) d ^= gmul(lam[i], s[r - i]);
        if (d == 0) { m++; continue; }
        uint8_t coef = gdiv(d, bd);
        if (2 * L <= r) {
            memcpy(t, lam, sizeof(t));
            for (int i = 0; i + m <= MAXR; i++) lam[i + m] ^= gmul(coef, b[i]);
            L = r + 1 - L;
            memcpy(b, t, sizeof(b));
            bd = d;
            m = 1;
        } else {
            for (int i = 0; i + m <= MAXR; i++) lam[i + m] ^= gmul(coef, b[i]);
            m++;
        }
    }
    if (L == 0 || L > n / 2) return -1;

    /* Ω = S·Λ mod x^n */
    uint8_t om[MAXR] = {0};
    for (int k = 0; k < n; k++)
        for (int j = 0; j <= k && j <= L; j++)
            om[k] ^= gmul(s[k - j], lam[j]);

    /* Chien + Forney (fcr=0: e = X · Ω(X⁻¹) / Λ'(X⁻¹)) */
    int found = 0;
    int pos[MAXR]; uint8_t mag[MAXR];
    for (int e = 0; e < blen; e++) {
        int inv = (255 - e) % 255;                  /* log(X⁻¹) */
        uint8_t v = 0;
        for (int i = 0; i <= L; i++)
            if (lam[i]) v ^= gexp[(glog[lam[i]] + inv * i) % 255];
        if (v) continue;
        if (found >= L) return -1;
        uint8_t o = 0;
        for (int i = 0; i < n; i++)
            if (om[i]) o ^= gexp[(glog[om[i]] + inv * i) % 255];
        uint8_t dl = 0;                              /* Λ'(x) = Σ_{i impar} λi x^(i-1) */
        for (int i = 1; i <= L; i += 2)
            if (lam[i]) dl ^= gexp[(glog[lam[i]] + inv * (i - 1)) % 255];
        if (!dl) return -1;
        pos[found] = blen - 1 - e;
        mag[found] = gmul(gexp[e % 255], gdiv(o, dl));
        found++;
    }
    if (found != L) return -1;
    for (int k = 0; k < found; k++) block[pos[k]] ^= mag[k];

    syndromes(block, blen, n, s);
    for (int i = 0; i < n; i++) if (s[i]) return -1;
    return found;
}
