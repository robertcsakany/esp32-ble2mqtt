#ifndef UART_EFM8B1_H
#define UART_EFM8B1_H

#include <stddef.h>
#include <stdint.h>

typedef void (*uart_efm88b1_raw_received_cb_t)(unsigned char *raw);
typedef void (*uart_efm88b1_known_received_cb_t)(unsigned char *raw);

void uart_efm88b1_set_raw_received_cb(uart_efm88b1_raw_received_cb_t cb);

void uart_efm88b1_set_known_received_cb(uart_efm88b1_known_received_cb_t cb);

int uart_efm88b1_init();

int uart_efm88b1_stop();

void uart_efm88b1_send(uint8_t *message);

#endif
