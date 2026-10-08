#include "dedup.h"
#include <string.h>
#include "esp_timer.h"

void dedup_init(dedup_t *d, dedup_entry_t *storage, int slots, int64_t ttl_us)
{
    d->entries = storage;
    d->slots   = slots;
    d->next    = 0;
    d->ttl_us  = ttl_us;
    memset(storage, 0, (size_t)slots * sizeof(*storage));
}

uint32_t dedup_hash(const uint8_t *data, size_t len)
{
    uint32_t h = 0x811c9dc5u;
    for (size_t i = 0; i < len; i++)
        h = (h ^ data[i]) * 0x01000193u;
    return h;
}

bool dedup_check_and_add(dedup_t *d, uint32_t hash)
{
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < d->slots; i++) {
        if (d->entries[i].ts_us != 0 && d->entries[i].hash == hash &&
            (now - d->entries[i].ts_us) < d->ttl_us)
            return true;
    }
    d->entries[d->next].hash  = hash;
    d->entries[d->next].ts_us = now ? now : 1;
    d->next = (d->next + 1) % d->slots;
    return false;
}
