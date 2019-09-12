#ifndef LIVOLO_BUCKET_CALC_H
#define LIVOLO_BUCKET_CALC_H

#include <stddef.h>
#include <stdint.h>

const char *livolo_calc_bucket(uint16_t remote_id, uint8_t key_code, uint8_t repeat, uint16_t pulse_start, uint16_t pulse_short, uint16_t pulse_long, char *buf);

#endif
