#pragma once
/* Tabla circular de hashes FNV-1a para suprimir tramas repetidas dentro de una
 * ventana temporal. Compartida por digipeater.c e il2p.c. No es thread-safe:
 * cada instancia debe usarse desde una sola tarea. */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct {
    uint32_t hash;
    int64_t  ts_us;   /* esp_timer_get_time() al insertar; 0 = slot libre */
} dedup_entry_t;

typedef struct {
    dedup_entry_t *entries;
    int            slots;
    int            next;     /* próximo slot a sobrescribir */
    int64_t        ttl_us;
} dedup_t;

void     dedup_init(dedup_t *d, dedup_entry_t *storage, int slots, int64_t ttl_us);
uint32_t dedup_hash(const uint8_t *data, size_t len);
/* true si el hash se vio dentro de la ventana; si no, lo registra y devuelve false. */
bool     dedup_check_and_add(dedup_t *d, uint32_t hash);
