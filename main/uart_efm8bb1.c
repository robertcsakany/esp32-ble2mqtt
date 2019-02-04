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


/*
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
*/

#define EX_UART_NUM UART_NUM_2
#define EX_UART UART2

#define ECHO_TEST_TXD  (GPIO_NUM_22)
#define ECHO_TEST_RXD  (GPIO_NUM_23)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)


#define BUF_SIZE (1024)
#define MAX_MESSAGE_SIZE (256)

static intr_handle_t handle_console;

// Receive buffer to collect incoming data
// uint8_t rxbuf[256];


static uart_efm88b1_raw_received_cb_t uart_efm88b1_raw_received_cb = NULL;

void uart_efm88b1_set_raw_received_cb(uart_efm88b1_raw_received_cb_t cb) {
	uart_efm88b1_raw_received_cb = cb;
}


void *bin2hex(char *hex, uint8_t *bin, int len)
{
    uint8_t *r = (uint8_t *)hex;

    while(len && p)
    {
        (*r) = ((*bin) & 0xF0) >> 4;
        (*r) = ((*r) <= 9 ? '0' + (*r) : 'A' - 10 + (*r));
        r++;
        (*r) = ((*bin) & 0x0F);
        (*r) = ((*r) <= 9 ? '0' + (*r) : 'A' - 10 + (*r));
        r++;
        bin++;
        len--;
    }
    *r = '\0';
}

void *hex2bin(uint8_t *bin, char *hex)
{
	int len, h;
    unsigned char *p, c;

    len = 0;
    p = (unsigned char*) hex;
    while (*p++)
        len++;

    h = !(len%2) * 4;
    p = bin;
    *p = 0;

    c = *hex;
    while(c)
    {
        if(('0' <= c) && (c <= '9'))
            *p += (c - '0') << h;
        else if(('A' <= c) && (c <= 'F'))
            *p += (c - 'A' + 10) << h;
        else if(('a' <= c) && (c <= 'f'))
            *p += (c - 'a' + 10) << h;

        hex++;
        c = *hex;

        if (h)
            h = 0;
        else
        {
            h = 4;
            p++;
            *p = 0;
        }
    }
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
//                case UART_PATTERN_DET:
//                    uart_get_buffered_data_len(uart_num, &buffered_size);
//                    int pos = uart_pattern_pop_pos(uart_num);
//                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
//                    if (pos == -1) {
//                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
//                        // record the position. We should set a larger queue size.
//                        // As an example, we directly flush the rx buffer here.
//                        uart_flush_input(uart_num);
//                    } else {
//                        uart_read_bytes(uart_num, dtmp, pos, 100 / portTICK_PERIOD_MS);
//                        uint8_t pat[PATTERN_CHR_NUM + 1];
//                        memset(pat, 0, sizeof(pat));
//                        uart_read_bytes(uart_num, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
//                        ESP_LOGI(TAG, "read data: %s", dtmp);
//                        ESP_LOGI(TAG, "read pat : %s", pat);
//                    }
//                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    //free(dtmp);
    //dtmp = NULL;
    //vTaskDelete(NULL);
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

/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
/*
static void IRAM_ATTR uart_intr_handle(void *arg)
{
  uint16_t rx_fifo_len;
  uint16_t i=0;

  //uint16_t status;
  //status = EX_UART.int_st.val; // read UART interrupt Status
  rx_fifo_len = EX_UART.status.rxfifo_cnt; // read number of bytes in UART buffer


  while(rx_fifo_len){
   rxbuf[i++] = EX_UART.fifo.rw_byte; // read all bytes
   rx_fifo_len--;
 }

 // after reading bytes from buffer clear UART interrupt status
 uart_clear_intr_status(EX_UART_NUM, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

//	uint8_t *hexstr = bin2hex(rxbuf, i);
//	ESP_LOGI(TAG, "Message get: %s", hexstr);

// a test code or debug code to indicate UART receives successfully,
// you can redirect received byte as echo also
 //uart_write_bytes(uart_num, (const char*) "RX Done", 7);
   uart_write_bytes(EX_UART_NUM,  (const char*) rxbuf, i);

}
*/

static void echo_task()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);

    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

	int current_length = 0;
    uint8_t *message_buff = (uint8_t *) malloc(MAX_MESSAGE_SIZE);
    uint8_t *hex = (uint8_t *) malloc(MAX_MESSAGE_SIZE * 2);


    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        uart_write_bytes(EX_UART_NUM, (const char *) data, len);

		if (len > 0) {

			ESP_LOGD(TAG, "Received bytes %d", len);

			for (int i = 0; i < len; i++) {
				uint8_t current_byte = data[i];
				message_buff[current_length++] = current_byte;

				// EOL, we got the whole messsage.
				if (current_byte == 55) {
					bin2hex(hex, message_buff, current_length);
					ESP_LOGI(TAG, "Message get: %s", hex);

					if (uart_efm88b1_raw_received_cb) {
						uart_efm88b1_raw_received_cb(hex);
					}
					current_length = 0;
				}
				if (current_length > MAX_MESSAGE_SIZE - 1) {
					ESP_LOGW(TAG, "Message overflow!");
					current_length = 0;
				}
			}
		}

    }
}

int uart_efm88b1_init()
{
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);


	/* Configure parameters of an UART driver,
	* communication pins and install the driver */
/*
uart_config_t uart_config = {
		.baud_rate = 19200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};

	ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart_config));

	//Set UART log level
	esp_log_level_set(TAG, ESP_LOG_INFO);

	//Set UART pins (using UART0 default pins ie no changes.)
	ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	//Install UART driver, and get the queue.
	ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

	// release the pre registered UART handler/subroutine
	ESP_ERROR_CHECK(uart_isr_free(EX_UART_NUM));

	// register new UART subroutine
	ESP_ERROR_CHECK(uart_isr_register(EX_UART_NUM,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console));

	// enable RX interrupt
	ESP_ERROR_CHECK(uart_enable_rx_intr(EX_UART_NUM));
*/

	/*
	if (uart_efm88b1_receive_data) {
		return -1;
	}

	ESP_LOGD(TAG, "Initializing UART EFM8BB1");


	uart_efm88b1_receive_data = (uint8_t *)malloc(BUF_SIZE);

	-/-*
	uart_config_t uart_config = {
			.baud_rate = 19200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
	        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
	        .rx_flow_ctrl_thresh = 122
	};
	-*=/


	uart_config_t uart_config = {
			.baud_rate = 19200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 0
	};

	ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD, RXD, RTS, CTS));

//	ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 0, &uart_efm88b1_queue, 0));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));

    //uint8_t *init = hex2bin("AAFF55");
    //uart_write_bytes(uart_num, (const char*) init, 3);

    ESP_ERROR_CHECK(uart_isr_free(uart_num));

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
	*/
	return 0;
}











