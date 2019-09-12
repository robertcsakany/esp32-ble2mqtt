#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "format_utils.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#define MAX_MESSAGE_SIZE (256)


/*
 From a byte array to an bin char array ("1000101...",  one byte is 8 chars)
 */
uint8_t bin2binary(uint8_t * in, char * out, int n) {
    for (unsigned char p = 0; p<n; p++) {
    	sprintf(&out[p*2], BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(in[p]));
    }
    return 1;
}


/*
 From a byte array to an hexa char array ("A220EE...", double the size)
 */
uint8_t bin2hex(uint8_t * in, char * out, int n) {
    for (unsigned char p = 0; p<n; p++) {
        sprintf(&out[p*2], "%02X", in[p]);
    }
    return 1;
}

/*
 From an hexa char array ("A220EE...") to a byte array (half the size)
 */
int hex2bin(const char * in, uint8_t * out, int length) {
    int n = strlen(in);
    if (n > MAX_MESSAGE_SIZE*2 || (length > 0 && n != length)) return 0;
    char tmp[3] = {0,0,0};
    n /= 2;
    for (unsigned char p = 0; p<n; p++) {
        memcpy(tmp, &in[p*2], 2);
        out[p] = strtol(tmp, NULL, 16);
    }
    return n;
}
