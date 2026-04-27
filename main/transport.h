#pragma once
#include <stdint.h>
#include <stddef.h>

// Interfaz que debe implementar cada transporte (WiFi TCP, UART, BT SPP…).
// init()  arranca el hardware y lanza las tareas internas del transporte.
// write() envía bytes al host; puede llamarse desde cualquier tarea.
typedef struct {
    void (*init)(void);
    void (*write)(const uint8_t *buf, size_t len);
} transport_ops_t;

// Registrar el transporte activo e inicializarlo.
void transport_init(const transport_ops_t *ops);

// Enviar bytes al host a través del transporte activo.
void transport_write(const uint8_t *buf, size_t len);
