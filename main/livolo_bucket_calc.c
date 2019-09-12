#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include "livolo_bucket_calc.h"
#include "format_utils.h"

void livolo_code_as_bin(uint16_t remote_id, uint8_t key_code, char *buf)
{
    int ofs = 0;
    uint32_t bits = (remote_id << 8) + key_code;

    buf[0] = '0';
    ofs++;
	for (int bit = 0; bit < 23; bit++) {
		if ((bits >> (22-bit)) & 1) {
			buf[bit+ofs] = '2';
		} else {
			buf[bit+ofs] = '1';
			buf[bit+ofs+1] = '1';
			ofs++;
		}
	}
    buf[ofs+24] = '\0';
}

// Caluculate RfRaw code for transmission.
// The UART payload:
//      Uart    Cmd   Data len   Num  Rep   Bucket0  Bucket1  Bucket2  Uart end
//                    8 + t                 550 us   110 us   303 us
//      AA      B0    1C         03   20    0226     006E     012F     55
// The 8 byte of datalen is Num (1 byte) + Rep (1 Byte) + Bucket1 size (2 byte) + Bucket2 size (2 byte) + Bucket3 size (2 byte)
//
// Example OFF / ON from: https://github.com/Portisch/RF-Bridge-EFM8BB1/issues/51
//
// int main()
// {
//    char buf[100];
//
//    printf("Buf: %s", lovolo_calc_bucket(7, 7, 50, 550, 110, 303, buf));
//
//    return 0;
//sss}

const char *livolo_calc_bucket(uint16_t remote_id, uint8_t key_code, uint8_t repeat, uint16_t pulse_start, uint16_t pulse_short, uint16_t pulse_long, char *buf)
{
    // The bucket_code is offseted
    char bucket_code[50]; // = &buf[18];

    // get remote_id, key_code as buckets of 1/2
	livolo_code_as_bin(remote_id, key_code, bucket_code);
	// Align code to a whole byte hex string
	if (strlen(bucket_code) % 2 == 1) {
	    bucket_code[strlen(bucket_code)] = 'F';
	    bucket_code[strlen(bucket_code)+1] = '\0';
	}
	// Calculate length field (1 byte 2 char)
	uint8_t len = 8+(strlen(bucket_code)/2);
	bin2hex(&len, buf, 1);

	// Constant - 3 buckets defined (1 byte 2 char)
	buf[2] = '0';
	buf[3] = '3';

	// Print Repeat field (1 byte 2 char)
	bin2hex(&repeat, &buf[4], 1);

	// Print Pulse start field (2 byte 4 char)
	bin2hex((uint8_t *)(&pulse_start)+1, &buf[6], 1);
	bin2hex((uint8_t *)(&pulse_start), &buf[8], 1);

	// Print Pulse short field (2 byte 4 char)
	bin2hex((uint8_t *)(&pulse_short)+1, &buf[10], 1);
	bin2hex((uint8_t *)(&pulse_short), &buf[12], 1);

	// Print Pulse long field (2 byte 4 char)
	bin2hex((uint8_t *)(&pulse_long)+1, &buf[14], 1);
	bin2hex((uint8_t *)(&pulse_long), &buf[16], 1);

	memcpy(&buf[18], &bucket_code, 50);

	return buf;
}

