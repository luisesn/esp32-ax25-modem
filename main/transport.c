#include "transport.h"

static const transport_ops_t *s_ops;

void transport_init(const transport_ops_t *ops) {
    s_ops = ops;
    if (s_ops && s_ops->init)
        s_ops->init();
}

void transport_write(const uint8_t *buf, size_t len) {
    if (s_ops && s_ops->write)
        s_ops->write(buf, len);
}
