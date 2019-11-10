#ifndef RF433RX_H_
#define RF433RX_H_

#include <stdint.h>
#include <stdbool.h>

typedef void (*rf433rx_on_message_cb_t)(uint32_t value,
            uint8_t bit_length,
            uint32_t received_delay,
            uint8_t received_protocol);

void rf433rx_set_on_message_cb(rf433rx_on_message_cb_t cb);

int rf433rx_init(int gpio);
void rf433rx_receiver_stop();

#endif /* RF433RX_H_ */
