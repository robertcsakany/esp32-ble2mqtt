/**
 * Inspired by 
 *    Dominik Palo's ESP32 RF receiver (https://github.com/DominikPalo/esp32-rf-receiver) 
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "sdkconfig.h"
#include <esp_log.h>
#include "esp_timer.h"
#include "rf433rx.h"
#include <stdlib.h>
#include <string.h>
#include <endian.h>

/* Constants */
static const char *TAG = "RF433RX";

static rf433rx_on_message_cb_t onmessage_cb = NULL;

void rf433rx_set_on_message_cb(rf433rx_on_message_cb_t cb)
{
    onmessage_cb = cb;
}

// Number of maximum high/Low changes per packet.
// We can handle up to (unsigned long) => 32 bit * 2 H/L changes per bit + 2 for sync
#define rf433rx_max_changes 67

//variables to avoid duplicates
#define rf433rx_time_avoid_duplicate 3000 // if you want to avoid duplicate mqtt message received set this to > 0, the value is the time in milliseconds during which we don't publish duplicates

/**
 * Description of a single pulse, which consists of a high signal
 * whose duration is "high" times the base pulse length, followed
 * by a low signal lasting "low" times the base pulse length.
 * Thus, the pulse overall lasts (high+low)*pulse_length
 */
typedef struct rf433rx_high_low {
    uint8_t high;
    uint8_t low;
} rf433rx_high_low;

/**
 * A "rf433rx_protocol" describes how zero and one bits are encoded into high/low
 * pulses.
 */
typedef struct rf433rx_protocol {
    /** base pulse length in microseconds, e.g. 350 */
    uint16_t pulse_length;

    rf433rx_high_low sync_factor;
    rf433rx_high_low zero;
    rf433rx_high_low one;

    /**
     * If true, interchange high and low logic levels in all transmissions.
     *
     * By default, RCSwitch assumes that any signals it sends or receives
     * can be broken down into pulses which start with a high signal level,
     * followed by a a low signal level. This is e.g. the case for the
     * popular PT 2260 encoder chip, and thus many switches out there.
     *
     * But some devices do it the other way around, and start with a low
     * signal level, followed by a high signal level, e.g. the HT6P20B. To
     * accommodate this, one can set invertedSignal to true, which causes
     * RCSwitch to change how it interprets any rf433rx_high_low struct FOO: It will
     * then assume transmissions start with a low signal lasting
     * FOO.high*pulse_length microseconds, followed by a high signal lasting
     * FOO.low*pulse_length microseconds.
     */
    bool inverted_signal;
} rf433rx_protocol;



/* Format for protocol definitions:
 * {pulselength, Sync bit, "0" bit, "1" bit}
 *
 * pulselength: pulse length in microseconds, e.g. 350
 * Sync bit: {1, 31} means 1 high pulse and 31 low pulses
 *     (perceived as a 31*pulselength long pulse, total length of sync bit is
 *     32*pulselength microseconds), i.e:
 *      _
 *     | |_______________________________ (don't count the vertical bars)
 * "0" bit: waveform for a data bit of value "0", {1, 3} means 1 high pulse
 *     and 3 low pulses, total length (1+3)*pulselength, i.e:
 *      _
 *     | |___
 * "1" bit: waveform for a data bit of value "1", e.g. {3,1}:
 *      ___
 *     |   |_
 *
 * These are combined to form Tri-State bits when sending or receiving codes.
 */
static const rf433rx_protocol proto[] =
{
		{ 350, { 1, 31 }, { 1, 3 }, { 3, 1 }, false },		// protocol 1
		{ 650, { 1, 10 }, { 1, 2 }, { 2, 1 }, false },		// protocol 2
		{ 100, { 30, 71 }, { 4, 11 }, { 9, 6 }, false },	// protocol 3
		{ 380, { 1, 6 }, { 1, 3 }, { 3, 1 }, false },		// protocol 4
		{ 500, { 6, 14 }, { 1, 2 }, { 2, 1 }, false },		// protocol 5
		{ 450, { 23, 1 }, { 1, 2 }, { 2, 1 }, true },		// protocol 6 (HT6P20B)
		{ 150, { 2, 62 }, { 1, 6 }, { 6, 1 }, false }		// protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
};

enum
{
	numProto = sizeof(proto) / sizeof(proto[0])
};

/* Internal state */
static volatile unsigned long rf433rx_received_value = 0;
static volatile unsigned int rf433rx_received_bit_length = 0;
static volatile unsigned int rf433rx_received_delay = 0;
static volatile unsigned int rf433rx_received_protocol = 0;
static int rf433rx_reveive_tolerance = 60;
const unsigned rf433rx_separation_limit = 4300;


// TaskHandle_t rf433rx_receive_task;
static uint64_t rf433rx_receive_pin_selector = ((uint64_t) (((uint64_t) 1) << 22));
static uint64_t rf433rx_receive_pin = 22;

// separationLimit: minimum microseconds between received codes, closer codes are ignored.
// according to discussion on issue #14 it might be more suitable to set the separation
// limit to the same time as the 'low' part of the sync signal for the current protocol.
static unsigned int timings[rf433rx_max_changes];

/* helper function for the rf433rx_receive_protocol method */
static inline unsigned int diff(int a, int b)
{
	return abs(a - b);
}

bool rf433rx_receive_protocol(const int p, unsigned int change_count)
{
	const rf433rx_protocol pro = proto[p - 1];

	unsigned long code = 0;
	//Assuming the longer pulse length is the pulse captured in timings[0]
	const unsigned int sync_length_in_pulses =
			((pro.sync_factor.low) > (pro.sync_factor.high)) ?
					(pro.sync_factor.low) : (pro.sync_factor.high);
	const unsigned int delay = timings[0] / sync_length_in_pulses;
	const unsigned int delay_tolerance = delay * rf433rx_reveive_tolerance / 100;

	/* For protocols that start low, the sync period looks like
	 *               _________
	 * _____________|         |XXXXXXXXXXXX|
	 *
	 * |--1st dur--|-2nd dur-|-Start data-|
	 *
	 * The 3rd saved duration starts the data.
	 *
	 * For protocols that start high, the sync period looks like
	 *
	 *  ______________
	 * |              |____________|XXXXXXXXXXXXX|
	 *
	 * |-filtered out-|--1st dur--|--Start data--|
	 *
	 * The 2nd saved duration starts the data
	 */
	const unsigned int first_data_timing = (pro.inverted_signal) ? (2) : (1);

	for (unsigned int i = first_data_timing; i < change_count - 1; i += 2)
	{
		code <<= 1;
		if (diff(timings[i], delay * pro.zero.high) < delay_tolerance
				&& diff(timings[i + 1], delay * pro.zero.low)
						< delay_tolerance)
		{
			// zero
		} else if (diff(timings[i], delay * pro.one.high) < delay_tolerance
				&& diff(timings[i + 1], delay * pro.one.low)
						< delay_tolerance)
		{
			// one
			code |= 1;
		} else {
			// Failed
			return false;
		}
	}

	if (change_count > 7)
	{ // ignore very short transmissions: no device sends them, so this must be noise
		rf433rx_received_value = code;
		rf433rx_received_bit_length = (change_count - 1) / 2;
		rf433rx_received_delay = delay;
		rf433rx_received_protocol = p;
		return true;
	}

	return false;
}

bool rf433rx_available()
{
	return rf433rx_received_value != 0;
}

void rf433rx_reset_available()
{
	rf433rx_received_value = 0;
}

unsigned long rf433rx_get_received_value()
{
	return rf433rx_received_value;
}

unsigned int rf433rx_get_received_bit_length()
{
	return rf433rx_received_bit_length;
}

unsigned int rf433rx_get_received_delay()
{
	return rf433rx_received_delay;
}

unsigned int rf433rx_get_received_protocol()
{
	return rf433rx_received_protocol;
}

unsigned int* rf433rx_get_received_rawdata()
{
	return timings;
}

#define rf433rx_array_size 12
uint32_t rf433rx_received_signal[rf433rx_array_size][2] ={{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};

int rf433rx_get_min()
{
  uint32_t minimum = rf433rx_received_signal[0][1];
  int minindex = 0;
  for (int i = 0; i < rf433rx_array_size; i++)
  {
    if (rf433rx_received_signal[i][1] < minimum) 
	{
      minimum = rf433rx_received_signal[i][1];
      minindex = i;
    }
  }
  return minindex;
}

void rf433rx_store_value(uint32_t value){
    uint32_t now =  esp_timer_get_time() / 1000;
    
	// find oldest value of the buffer
    int o = rf433rx_get_min();

    // replace it by the new one
    rf433rx_received_signal[o][0] = value;
    rf433rx_received_signal[o][1] = now;
}

bool rf433rx_is_duplicated(unsigned long value)
{
	// check if the value has been already sent during the last time_avoid_duplicate
	for (int i = 0; i < rf433rx_array_size;i++)
	{
		if (rf433rx_received_signal[i][0] == value)
		{
			uint32_t now = esp_timer_get_time() / 1000;
			if (now - rf433rx_received_signal[i][1] < rf433rx_time_avoid_duplicate)
			{
				// change
				return true;
			}
		}
	}
	return false;
}


// ---
void rf433rx_data_interrupt_handler(void* arg)
{
	static unsigned int change_count = 0;
	static unsigned long lastTime = 0;
	static unsigned int repeatCount = 0;

	const long time = esp_timer_get_time();
	const unsigned int duration = time - lastTime;

	if (duration > rf433rx_separation_limit)
	{
		// A long stretch without signal level change occurred. This could
		// be the gap between two transmission.
		if (diff(duration, timings[0]) < 200)
		{
			// This long signal is close in length to the long signal which
			// started the previously recorded timings; this suggests that
			// it may indeed by a a gap between two transmissions (we assume
			// here that a sender will send the signal multiple times,
			// with roughly the same gap between them).
			repeatCount++;
			if (repeatCount == 2)
			{
				for (uint8_t i = 1; i <= numProto; i++)
				{
					if (rf433rx_receive_protocol(i, change_count))
					{
						// receive succeeded for protocol i
						uint8_t protocol_num = (uint8_t) i;
				        if (onmessage_cb && !rf433rx_is_duplicated(rf433rx_get_received_value())) 
        				{
							onmessage_cb(
									(uint32_t) rf433rx_get_received_value(), 
									(uint8_t) rf433rx_get_received_bit_length(), 
									(uint32_t) rf433rx_get_received_delay(), 
									(uint8_t) protocol_num									
							);
							rf433rx_store_value((uint32_t) rf433rx_get_received_value());
							rf433rx_reset_available();
						}
						break;
					}
				}
				repeatCount = 0;
			}
		}
		change_count = 0;
	}
	// detect overflow
	if (change_count >= rf433rx_max_changes)
	{
		change_count = 0;
		repeatCount = 0;
	}

	timings[change_count++] = duration;
	lastTime = time;
}

int rf433rx_init(int gpio)
{
    ESP_LOGD(TAG, "Initializing 433Mhz receiver");

	rf433rx_receive_pin = gpio;
	rf433rx_receive_pin_selector = ((uint64_t) (((uint64_t) 1) << gpio));

	gpio_config_t data_pin_config =
	{
			.intr_type = GPIO_INTR_ANYEDGE,
			.mode = GPIO_MODE_INPUT,
			.pin_bit_mask = rf433rx_receive_pin_selector,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE
	};
	gpio_config(&data_pin_config);

	// Attach the interrupt handler
	gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
	gpio_isr_handler_add(gpio, rf433rx_data_interrupt_handler, NULL);

	return 0;
}

void rf433rx_receiver_stop()
{
	gpio_isr_handler_remove(rf433rx_receive_pin);
}
