#ifndef FORMAT_UTILS_H
#define FORMAT_UTILS_H

#include <stddef.h>
#include <stdint.h>

uint8_t bin2binary(uint8_t * in, char * out, int n);
uint8_t bin2hex(uint8_t * in, char * out, int n);
int hex2bin(const char * in, uint8_t * out, int length);

#endif
