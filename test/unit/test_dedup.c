/* Tests de main/dedup.c (tabla FNV-1a con TTL y reloj falso de esp_timer). */
#include "test.h"
#include "esp_timer.h"
#include "dedup.h"

#define SLOTS 4
#define TTL_US 1000000   /* 1 s */

static dedup_entry_t store[SLOTS];
static dedup_t d;

static void setup(void)
{
    test_time_set_us(1000000);             /* reloj no nulo: ts 0 significa slot libre */
    dedup_init(&d, store, SLOTS, TTL_US);
}

TEST(hash_known_values)
{
    /* FNV-1a 32 bits de referencia: "" = offset basis; "a" = 0xE40C292C; "foobar" = 0xBF9CF968. */
    CHECK(dedup_hash((const uint8_t *)"", 0) == 0x811c9dc5u);
    CHECK(dedup_hash((const uint8_t *)"a", 1) == 0xE40C292Cu);
    CHECK(dedup_hash((const uint8_t *)"foobar", 6) == 0xBF9CF968u);
    /* Longitud explícita: sólo cuenta el prefijo indicado */
    CHECK(dedup_hash((const uint8_t *)"abc", 1) == dedup_hash((const uint8_t *)"a", 1));
}

TEST(first_seen_then_duplicate)
{
    setup();
    uint32_t h = dedup_hash((const uint8_t *)"frame-1", 7);
    CHECK(dedup_check_and_add(&d, h) == false);
    CHECK(dedup_check_and_add(&d, h) == true);
    test_time_advance_us(TTL_US / 2);
    CHECK(dedup_check_and_add(&d, h) == true);
}

TEST(ttl_expires)
{
    setup();
    uint32_t h = 0xDEADBEEFu;
    CHECK(dedup_check_and_add(&d, h) == false);
    test_time_advance_us(TTL_US - 1);
    CHECK(dedup_check_and_add(&d, h) == true);     /* aún dentro: no refresca ts */
    test_time_advance_us(1);                       /* ahora exactamente TTL desde la inserción */
    CHECK(dedup_check_and_add(&d, h) == false);    /* expirado: se vuelve a aceptar */
    CHECK(dedup_check_and_add(&d, h) == true);
}

TEST(distinct_hashes_independent)
{
    setup();
    CHECK(dedup_check_and_add(&d, 1u) == false);
    CHECK(dedup_check_and_add(&d, 2u) == false);
    CHECK(dedup_check_and_add(&d, 1u) == true);
    CHECK(dedup_check_and_add(&d, 2u) == true);
}

TEST(capacity_eviction_is_circular)
{
    setup();
    /* Llenar los SLOTS con h=1..4; el quinto sobrescribe el slot 0 (h=1). */
    for (uint32_t h = 1; h <= SLOTS; h++) CHECK(dedup_check_and_add(&d, h) == false);
    CHECK(dedup_check_and_add(&d, 100u) == false);  /* evicta h=1 */
    CHECK(dedup_check_and_add(&d, 1u) == false);    /* h=1 ya no está: se registra de nuevo (evicta h=2) */
    CHECK(dedup_check_and_add(&d, 3u) == true);     /* h=3 sigue en tabla */
    CHECK(dedup_check_and_add(&d, 4u) == true);
    CHECK(dedup_check_and_add(&d, 100u) == true);
    CHECK_EQ_INT(d.next, 2);                        /* índice circular: 6 inserciones mod 4 */
}

TEST(eviction_wraps_under_churn)
{
    setup();
    /* Muchas inserciones distintas: nunca se sale del array, y el último hash sigue detectándose. */
    for (uint32_t h = 1; h <= 1000; h++) CHECK(dedup_check_and_add(&d, h) == false);
    CHECK(dedup_check_and_add(&d, 1000u) == true);
    CHECK(dedup_check_and_add(&d, 997u) == true);   /* 4 últimos: 997..1000 */
    CHECK(dedup_check_and_add(&d, 996u) == false);  /* ya evictado */
    CHECK(d.next >= 0 && d.next < SLOTS);
}

TEST(expired_slots_reused_without_false_positive)
{
    setup();
    CHECK(dedup_check_and_add(&d, 7u) == false);
    test_time_advance_us(2 * TTL_US);
    /* Una entrada expirada no debe contar como duplicado aunque el hash coincida. */
    CHECK(dedup_check_and_add(&d, 7u) == false);
    CHECK(dedup_check_and_add(&d, 7u) == true);
}

TEST(init_clears_storage_and_zero_time_slot)
{
    setup();
    CHECK(dedup_check_and_add(&d, 55u) == false);
    dedup_init(&d, store, SLOTS, TTL_US);           /* re-init borra la tabla */
    CHECK(dedup_check_and_add(&d, 55u) == false);
    /* Un hash de cero también se puede registrar (ts != 0 con reloj > 0). */
    CHECK(dedup_check_and_add(&d, 0u) == false);
    CHECK(dedup_check_and_add(&d, 0u) == true);
}

TEST(single_slot_table)
{
    dedup_entry_t one[1];
    dedup_t s;
    test_time_set_us(5000);
    dedup_init(&s, one, 1, TTL_US);
    CHECK(dedup_check_and_add(&s, 9u) == false);
    CHECK(dedup_check_and_add(&s, 9u) == true);
    CHECK(dedup_check_and_add(&s, 8u) == false);    /* sobrescribe el único slot */
    CHECK(dedup_check_and_add(&s, 9u) == false);
}

int main(void)
{
    RUN(hash_known_values);
    RUN(first_seen_then_duplicate);
    RUN(ttl_expires);
    RUN(distinct_hashes_independent);
    RUN(capacity_eviction_is_circular);
    RUN(eviction_wraps_under_churn);
    RUN(expired_slots_reused_without_false_positive);
    RUN(init_clears_storage_and_zero_time_slot);
    RUN(single_slot_table);
    return test_summary();
}
