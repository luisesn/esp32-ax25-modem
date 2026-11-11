// Mini framework de tests: sin dependencias. Uso:
//   TEST(nombre) { CHECK(x == 1); CHECK_EQ_INT(a, b); CHECK_MEM(a, b, n); }
//   int main(void) { RUN(nombre); return test_summary(); }
#pragma once
#include <stdio.h>
#include <string.h>
#include <stdint.h>

static int t_fail = 0, t_checks = 0, t_tests = 0, t_cur_fail = 0;

#define TEST(name) static void name(void)
#define RUN(name) do { t_cur_fail = 0; t_tests++; name(); \
    printf("[%s] %s\n", t_cur_fail ? "FAIL" : " ok ", #name); } while (0)
#define CHECK(c) do { t_checks++; if (!(c)) { t_fail++; t_cur_fail++; \
    printf("   %s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); } } while (0)
#define CHECK_EQ_INT(a, b) do { long long _a = (long long)(a), _b = (long long)(b); t_checks++; \
    if (_a != _b) { t_fail++; t_cur_fail++; \
    printf("   %s:%d: %s=%lld != %s=%lld\n", __FILE__, __LINE__, #a, _a, #b, _b); } } while (0)
#define CHECK_STR(a, b) do { const char *_a = (a), *_b = (b); t_checks++; \
    if (!_a || !_b || strcmp(_a, _b)) { t_fail++; t_cur_fail++; \
    printf("   %s:%d: \"%s\" != \"%s\"\n", __FILE__, __LINE__, _a ? _a : "(null)", _b ? _b : "(null)"); } } while (0)
#define CHECK_MEM(a, b, n) do { t_checks++; if (memcmp((a), (b), (n))) { t_fail++; t_cur_fail++; \
    printf("   %s:%d: memoria distinta (%s vs %s, %d B)\n", __FILE__, __LINE__, #a, #b, (int)(n)); } } while (0)

// Fallo conocido de producción: no rompe el test; avisa si ya se arregló.
#define KNOWN_BUG(c, why) do { if (c) printf("   %s:%d: KNOWN_BUG corregido, quitar la marca (%s)\n", __FILE__, __LINE__, why); \
    else t_known++; } while (0)
static int t_known = 0;

static inline int test_summary(void) {
    printf("%d tests, %d checks, %d fallos, %d fallos conocidos\n", t_tests, t_checks, t_fail, t_known);
    return t_fail ? 1 : 0;
}
