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

#define TXD  (17)
#define RXD  (16)
#define RTS  (18)
#define CTS  (19)
const int uart_num = UART_NUM_2;
#define BUF_SIZE (128)
#define RD_BUF_SIZE (BUF_SIZE)

TaskHandle_t efm8bb_uart_receive_task;
uint8_t *uart_efm88b1_receive_data = NULL;
static uart_efm88b1_raw_received_cb_t uart_efm88b1_raw_received_cb = NULL;
QueueHandle_t uart_efm88b1_queue;


void uart_efm88b1_set_raw_received_cb(uart_efm88b1_raw_received_cb_t cb) {
	uart_efm88b1_raw_received_cb = cb;
}


uint8_t *bin2hex(uint8_t *p, int len)
{
    static uint8_t hex[BUF_SIZE * 2];
	// char *hex = malloc(((2*len) + 1));


    uint8_t *r = hex;

    while(len && p)
    {
        (*r) = ((*p) & 0xF0) >> 4;
        (*r) = ((*r) <= 9 ? '0' + (*r) : 'A' - 10 + (*r));
        r++;
        (*r) = ((*p) & 0x0F);
        (*r) = ((*r) <= 9 ? '0' + (*r) : 'A' - 10 + (*r));
        r++;
        p++;
        len--;
    }
    *r = '\0';

    return hex;
}

uint8_t *hex2bin(const char *str)
{
    static uint8_t result[BUF_SIZE];

	int len, h;
    unsigned char *err, *p, c;

    err = malloc(1);
    *err = 0;

    if (!str)
        return err;

    if (!*str)
        return err;

    len = 0;
    p = (unsigned char*) str;
    while (*p++)
        len++;

    // result = malloc((len/2)+1);
    h = !(len%2) * 4;
    p = result;
    *p = 0;

    c = *str;
    while(c)
    {
        if(('0' <= c) && (c <= '9'))
            *p += (c - '0') << h;
        else if(('A' <= c) && (c <= 'F'))
            *p += (c - 'A' + 10) << h;
        else if(('a' <= c) && (c <= 'f'))
            *p += (c - 'a' + 10) << h;
        else
            return err;

        str++;
        c = *str;

        if (h)
            h = 0;
        else
        {
            h = 4;
            p++;
            *p = 0;
        }
    }

    return result;
}

/*
static void uart_efm88b1_receiver(void *pvParameters)
{
    uart_event_t event;
    // size_t buffered_size;
    static uint8_t dtmp[RD_BUF_SIZE];
    //= (uint8_t*) malloc(RD_BUF_SIZE);

	int current_length = 0;
    static uint8_t message_buff[BUF_SIZE];


    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_efm88b1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", uart_num);
            switch(event.type) {
                //Event of UART receving data
                // We'd better handler data event fast, there would be much more data events than
                // other types of events. If we take too much time on data event, the queue might
                // be full.
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    int len = uart_read_bytes(uart_num, dtmp, event.size, portMAX_DELAY);

            		if (len > 0) {
            			for (int i = 0; i < len; i++) {
            				uint8_t current_byte = uart_efm88b1_receive_data[i];
            				message_buff[current_length] = current_byte;
            				current_length++;

            				// EOL, we got the whole messsage.
            				if (current_byte == 55) {
            					uint8_t *hexstr = bin2hex(message_buff, current_length);
            					ESP_LOGI(TAG, "Message get: %s", hexstr);

            					if (uart_efm88b1_raw_received_cb) {
            						uart_efm88b1_raw_received_cb(hexstr);
            					}
            					current_length = 0;
            				}
            				if (current_length > sizeof(message_buff) - 1) {
            					ESP_LOGW(TAG, "Message overflow!");
            					current_length = 0;
            				}
            			}
            		}

                    // ESP_LOGI(TAG, "[DATA EVT]:");
                    // uart_write_bytes(uart_num, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uart_num);
                    xQueueReset(uart_efm88b1_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uart_num);
                    xQueueReset(uart_efm88b1_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(uart_num, &buffered_size);
                    int pos = uart_pattern_pop_pos(uart_num);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(uart_num);
                    } else {
                        uart_read_bytes(uart_num, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(uart_num, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    //free(dtmp);
    //dtmp = NULL;
    vTaskDelete(NULL);
}
*/
/*
void uart_efm88b1_receiver(void* pv_parameter)
{
	int current_length = 0;
    static uint8_t message_buff[BUF_SIZE];

	while (1) {
		int len = uart_read_bytes(uart_num, uart_efm88b1_receive_data, BUF_SIZE, 20 / portTICK_RATE_MS);

		if (len > 0) {

			ESP_LOGD(TAG, "Received bytes %d", len);

			for (int i = 0; i < len; i++) {
				uint8_t current_byte = uart_efm88b1_receive_data[i];
				message_buff[current_length] = current_byte;
				current_length++;

				// EOL, we got the whole messsage.
				if (current_byte == 55) {
					uint8_t *hexstr = bin2hex(message_buff, current_length);
					ESP_LOGI(TAG, "Message get: %s", hexstr);

					if (uart_efm88b1_raw_received_cb) {
						uart_efm88b1_raw_received_cb(hexstr);
					}
					current_length = 0;
				}
				if (current_length > sizeof(message_buff) - 1) {
					ESP_LOGW(TAG, "Message overflow!");
					current_length = 0;
				}
			}
		}
	}
}
*/

int uart_efm88b1_init()
{
	if (uart_efm88b1_receive_data) {
		return -1;
	}

	ESP_LOGD(TAG, "Initializing UART EFM8BB1");


	uart_efm88b1_receive_data = (uint8_t *)malloc(BUF_SIZE);

	/*
	uart_config_t uart_config = {
			.baud_rate = 19200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
	        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
	        .rx_flow_ctrl_thresh = 122
	};
	*/

	/*
	uart_config_t uart_config = {
			.baud_rate = 19200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 0
	};

	uart_param_config(uart_num, &uart_config);
	uart_set_pin(uart_num, TXD, RXD, RTS, CTS);

	ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 0, &uart_efm88b1_queue, 0));

    //uint8_t *init = hex2bin("AAFF55");
    //uart_write_bytes(uart_num, (const char*) init, 3);

	xTaskCreate(&uart_efm88b1_receiver, "uart_efm88b1_receiver", 2048, NULL, 12, &efm8bb_uart_receive_task);
	*/
	return 0;
}

int uart_efm88b1_stop()
{
	/*
	vTaskDelete(efm8bb_uart_receive_task);
	uart_driver_delete(uart_num);
	free(uart_efm88b1_receive_data);
	uart_efm88b1_receive_data = NULL;
	return 0;
	*/
}











