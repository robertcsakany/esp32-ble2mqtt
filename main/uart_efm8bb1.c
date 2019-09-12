#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "config.h"
#include "format_utils.h"
#include "uart_efm8bb1.h"

/* Constants */
static const char *TAG = "EFM8BB1";

#define BUF_SIZE (1024)
#define MAX_MESSAGE_SIZE (256)


// EFM8 Protocol
#define RF_MESSAGE_SIZE         9
#define RF_MAX_MESSAGE_SIZE     (112+4)
#define RF_CODE_START           0xAA
#define RF_CODE_ACK             0xA0
#define RF_CODE_LEARN           0xA1
#define RF_CODE_LEARN_KO        0xA2
#define RF_CODE_LEARN_OK        0xA3
#define RF_CODE_RFIN            0xA4
#define RF_CODE_RFOUT           0xA5
#define RF_CODE_SNIFFING_ON     0xA6
#define RF_CODE_SNIFFING_OFF    0xA7
#define RF_CODE_RFOUT_NEW       0xA8
#define RF_CODE_LEARN_NEW       0xA9
#define RF_CODE_LEARN_KO_NEW    0xAA
#define RF_CODE_LEARN_OK_NEW    0xAB
#define RF_CODE_RFOUT_BUCKET    0xB0
#define RF_CODE_STOP            0x55
#define RF_MAX_KEY_LENGTH       (9)

static uart_efm88b1_raw_received_cb_t uart_efm88b1_raw_received_cb = NULL;
static uart_efm88b1_known_received_cb_t uart_efm88b1_known_received_cb = NULL;

uart_port_t uart_num = UART_NUM_2;
int uart_txd_pin = 22;
int uart_rxd_pin = 23;

void uart_efm88b1_set_raw_received_cb(uart_efm88b1_raw_received_cb_t cb) {
	uart_efm88b1_raw_received_cb = cb;
}

void _uart_efm88b1_write(uint8_t code) {
    uart_write_bytes(uart_num, (const char *) &code, 1);
}

void _uart_efm88b1_ack() {
	_uart_efm88b1_write(RF_CODE_START);
	_uart_efm88b1_write(RF_CODE_ACK);
	_uart_efm88b1_write(RF_CODE_STOP);
}

void _uart_efm88b1_learn() {
	_uart_efm88b1_write(RF_CODE_START);
	_uart_efm88b1_write(RF_CODE_LEARN);
    _uart_efm88b1_write(RF_CODE_STOP);
}

void _uart_efm88b1_send_raw(uint8_t *message, int n) {
    for (int j=0; j<n; j++) {
    	_uart_efm88b1_write(message[j]);
    }
}

// The standard transmit command for PT2260, PT2262, PT2264, EV1527, etc devices.
// https://github.com/Portisch/RF-Bridge-EFM8BB1/wiki/0xA5
void _uart_efm88b1_send_PT2260(uint8_t *message) {
	_uart_efm88b1_write(RF_CODE_START);
	_uart_efm88b1_write(RF_CODE_RFOUT);
	_uart_efm88b1_send_raw(message, strlen(message));
	_uart_efm88b1_write(RF_CODE_STOP);
}

// Have to be in form of https://github.com/Portisch/RF-Bridge-EFM8BB1/wiki/0xB0
void _uart_efm88b1_send_bucket(uint8_t *message) {
	_uart_efm88b1_write(RF_CODE_START);
	_uart_efm88b1_write(RF_CODE_RFOUT_BUCKET);
	_uart_efm88b1_send_raw(message, strlen(message));
	_uart_efm88b1_write(RF_CODE_STOP);
}

void _uart_efm88b1_send_raw_once(uint8_t *code, unsigned char length) {
    char buffer[length*2];
    bin2hex(code, buffer, length);
    _uart_efm88b1_send_raw(code, length);
}


void _uart_efm88b1_decode(uint8_t *message_buff) {
    uint8_t action = message_buff[0];
    unsigned char buffer[RF_MESSAGE_SIZE * 2 + 1] = {0};

    ESP_LOGI(TAG, "Action 0x%02X", action);
    if (action == RF_CODE_LEARN_KO) {
        _uart_efm88b1_ack();
        ESP_LOGI(TAG, "Learn timeout");
    }

    if (action == RF_CODE_LEARN_OK || action == RF_CODE_RFIN) {
        _uart_efm88b1_ack();
        bin2hex(&message_buff[1], (char *)buffer, RF_MESSAGE_SIZE);
        ESP_LOGI(TAG, "Received message '%s'", buffer);
    }

    if (action == RF_CODE_LEARN_OK) {
        ESP_LOGI(TAG, "Learn success");
        // TODO: Store learnt code
        // rfbStore(_learnId, _learnStatus, buffer);
    }

    if (action == RF_CODE_RFIN) {

        /* Look for the code, possibly replacing the code with the exact learned one on match
         * we want to do this on learn too to be sure that the learned code is the same if it
         * is equivalent
         */
    	/*
        unsigned char id;
        unsigned char status;
        // TODO get matched code
        bool matched = _rfbMatch(buffer, id, status, buffer);

        if (matched) {
            ESP_LOGI(TAG, "Matched message '%s'", buffer);
            _rfbin = true;
            if (status == 2) {
                relayToggle(id);
            } else {
                relayStatus(id, status == 1);
            }
        }
        */

		if (uart_efm88b1_raw_received_cb) {
			uart_efm88b1_raw_received_cb(buffer);
		}

    }

}

TaskHandle_t uart_efm88b1_task_handle;
uint8_t *uart_efm88b1_message_buff;
uint8_t *uart_efm88b1_uart_buff;

static void uart_efm88b1_task()
{
    uint8_t current_length = 0;
    uint8_t receiving = 0;

    while (1) {
        int len = uart_read_bytes(uart_num, uart_efm88b1_uart_buff, BUF_SIZE, 20 / portTICK_RATE_MS);
		if (len > 0) {

			ESP_LOGD(TAG, "Received bytes %d", len);

			for (int i = 0; i < len; i++) {
				uint8_t current_byte = uart_efm88b1_uart_buff[i];
				if (receiving) {

					if (current_byte == RF_CODE_STOP && (current_length == 1 || current_length == RF_MESSAGE_SIZE + 1)) {
						_uart_efm88b1_decode(uart_efm88b1_message_buff);
						receiving = 0;
					} else if (current_length <= RF_MESSAGE_SIZE) {
						uart_efm88b1_message_buff[current_length++] = current_byte;
					} else {
						receiving = 0;
					}
				} else if (current_byte == RF_CODE_START) {
					current_length = 0;
					receiving = 1;
				}
			}
		}

    }
}

/**
 * UART driver to handle UART interrupt.
 *
 * - Port: UART2
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */
int uart_efm88b1_init()
{
	uart_efm88b1_message_buff = (uint8_t *) malloc(MAX_MESSAGE_SIZE);
	uart_efm88b1_uart_buff = (uint8_t *) malloc(BUF_SIZE);

	uart_num = config_efm8bb1_uart_num_get(uart_num);
	uart_rxd_pin = config_efm8bb1_uart_rxpin_get(uart_rxd_pin);
	uart_txd_pin = config_efm8bb1_uart_txpin_get(uart_txd_pin);

    ESP_LOGI(TAG, "Initialize EFM88B1 Uart num: %d TX pin: %d RX pin: %d ", uart_num, uart_txd_pin, uart_rxd_pin);


    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, uart_txd_pin, uart_rxd_pin, -1, -1));

    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

    xTaskCreate(uart_efm88b1_task, "uart_efm88b1_task", 2048, NULL, 10, &uart_efm88b1_task_handle);
	return 0;
}

void uart_efm88b1_send_bucket(uint8_t *message) {
	_uart_efm88b1_send(message);
}

void uart_efm88b1_send_PT2260(uint16_t tsyn, uint16_t tlow, uint16_t thigh, uint32_t data) {

    char code[20];

	// Print tsyn (2 byte 4 char)
	bin2hex((uint8_t *)(&tsyn)+1, &buf[0], 1);
	bin2hex((uint8_t *)(&tsyn), &buf[2], 1);

	// Print tlow (2 byte 4 char)
	bin2hex((uint8_t *)(&tlow)+1, &buf[4], 1);
	bin2hex((uint8_t *)(&tlow), &buf[6], 1);

	// Print thigh (2 byte 4 char)
	bin2hex((uint8_t *)(&thigh)+1, &buf[8], 1);
	bin2hex((uint8_t *)(&thigh), &buf[10], 1);

	// Print data (3 byte 6 char)
	bin2hex((uint8_t *)(&data)+2, &buf[12], 1);
	bin2hex((uint8_t *)(&data)+1, &buf[14], 1);
	bin2hex((uint8_t *)(&data), &buf[16], 1);

	buf[18] = '\0';

	_uart_efm88b1_send_PT2260(code);
}

int uart_efm88b1_stop()
{
	vTaskDelete(uart_efm88b1_task_handle);
	ESP_ERROR_CHECK(uart_driver_delete(uart_num));
	free(uart_efm88b1_message_buff);
	free(uart_efm88b1_uart_buff);
	uart_efm88b1_message_buff = NULL;
	uart_efm88b1_uart_buff = NULL;
	return 0;
}











