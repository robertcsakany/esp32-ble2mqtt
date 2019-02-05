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

#include "uart_efm8bb1.h"

/* Constants */
static const char *TAG = "EFM8BB1";


#define EX_UART_NUM UART_NUM_2
#define EX_UART UART2

#define ECHO_TEST_TXD  (GPIO_NUM_22)
#define ECHO_TEST_RXD  (GPIO_NUM_23)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)


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


void uart_efm88b1_set_raw_received_cb(uart_efm88b1_raw_received_cb_t cb) {
	uart_efm88b1_raw_received_cb = cb;
}

/*
 From a byte array to an hexa char array ("A220EE...", double the size)
 */
static uint8_t _bin2hex(uint8_t * in, char * out, int n) {
    for (unsigned char p = 0; p<n; p++) {
        sprintf(&out[p*2], "%02X", in[p]);
    }
    return 1;
}

/*
 From an hexa char array ("A220EE...") to a byte array (half the size)
 */
static int _hex2bin(const char * in, uint8_t * out, int length) {
    int n = strlen(in);
    if (n > RF_MAX_MESSAGE_SIZE*2 || (length > 0 && n != length)) return 0;
    char tmp[3] = {0,0,0};
    n /= 2;
    for (unsigned char p = 0; p<n; p++) {
        memcpy(tmp, &in[p*2], 2);
        out[p] = strtol(tmp, NULL, 16);
    }
    return n;
}

void _uart_efm88b1_write(uint8_t code) {
    uart_write_bytes(EX_UART_NUM, (const char *) &code, 1);
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

void _uart_efm88b1_send(uint8_t *message) {
	_uart_efm88b1_write(RF_CODE_START);
	_uart_efm88b1_write(RF_CODE_RFOUT);
	_uart_efm88b1_send_raw(message, RF_MESSAGE_SIZE);
	_uart_efm88b1_write(RF_CODE_STOP);
}

void _uart_efm88b1_send_raw_once(uint8_t *code, unsigned char length) {
    char buffer[length*2];
    _bin2hex(code, buffer, length);
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
        _bin2hex(&message_buff[1], (char *)buffer, RF_MESSAGE_SIZE);
        ESP_LOGI(TAG, "Received message '%s'", buffer);
    }

    if (action == RF_CODE_LEARN_OK) {
        ESP_LOGI(TAG, "Learn success");
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
        int len = uart_read_bytes(EX_UART_NUM, uart_efm88b1_uart_buff, BUF_SIZE, 20 / portTICK_RATE_MS);
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

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    xTaskCreate(uart_efm88b1_task, "uart_efm88b1_task", 2048, NULL, 10, &uart_efm88b1_task_handle);
	return 0;
}

int uart_efm88b1_stop()
{
	vTaskDelete(uart_efm88b1_task_handle);
	ESP_ERROR_CHECK(uart_driver_delete(EX_UART_NUM));
	free(uart_efm88b1_message_buff);
	free(uart_efm88b1_uart_buff);
	uart_efm88b1_message_buff = NULL;
	uart_efm88b1_uart_buff = NULL;
	return 0;
}











