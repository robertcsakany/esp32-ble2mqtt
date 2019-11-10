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
#include <stdlib.h>
#include <string.h>
#include <endian.h>


/* Constants */
static const char *TAG = "LIVOLO";

#define livolotx_preamble_duration 525
#define livolotx_zero_duration 150
#define livolotx_one_duration 300
#define livolotx_num_repeats 150

#define NOP() asm volatile ("nop")

unsigned long IRAM_ATTR livolotx_micros()
{
    return (unsigned long) (esp_timer_get_time());
}

void IRAM_ATTR livolotx_delay(uint32_t us)
{
    uint32_t m = livolotx_micros();
    if (us) 
    {
        uint32_t e = (m + us);
        if (m > e)
        { //overflow
            while (livolotx_micros() > e)
            {
                NOP();
            }
        }
        while (livolotx_micros() < e)
        {
            NOP();
        }
    }
}

#define livolotx_tx(v) (gpio_set_level(livolotx_pin, v ? 1 : 0))

#define livolotx_send_one() \
{ \
  livolotx_delay(livolotx_one_duration); \
  livolotx_is_high = !livolotx_is_high; \
  livolotx_tx(livolotx_is_high); \
}


#define livolotx_send_zero() \
{ \
  livolotx_delay(livolotx_zero_duration); \
  livolotx_tx(!livolotx_is_high); \
  livolotx_delay(livolotx_zero_duration); \
  livolotx_tx(livolotx_is_high); \
}

#define livolotx_send_preamble() \
{ \
  livolotx_tx(true); \
  livolotx_delay(livolotx_preamble_duration); \
  livolotx_tx(false); \
  livolotx_is_high = false; \
}

static volatile bool livolotx_is_high = 0;

static uint64_t livolotx_pin_selector = ((uint64_t) (((uint64_t) 1) << 12));
static uint64_t livolotx_pin = 12;

int livolotx_init(int gpio)
{
  ESP_LOGD(TAG, "Initializing Livolo sender");

	livolotx_pin = gpio;
  livolotx_pin_selector = ((uint64_t) (((uint64_t) 1) << gpio));

  gpio_config_t livolotx_pin_conf;
  livolotx_pin_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  livolotx_pin_conf.mode = GPIO_MODE_OUTPUT;
  livolotx_pin_conf.pin_bit_mask = livolotx_pin_selector;
  livolotx_pin_conf.pull_down_en = 0;
  livolotx_pin_conf.pull_up_en = 0;
  gpio_config(&livolotx_pin_conf);

	return 0;
}

void IRAM_ATTR livolotx_send_command(uint32_t command, uint8_t num_bits)
{
  for (uint16_t repeat = 0; repeat < livolotx_num_repeats; ++repeat)
  {
    uint32_t mask = (1 << (num_bits - 1));
    livolotx_send_preamble();
    for (uint8_t i = num_bits; i > 0; --i)
    {
      if ((command & mask) > 0)
      {
        livolotx_send_one();
      }
      else
      {
        livolotx_send_zero();
      }
      mask >>= 1;
    }
  }
  livolotx_tx(false);
}

void livolotx_send(uint16_t remote_id, uint8_t key_id)
{
  // 7 bit Key Id and 16 bit Remote Id
  ESP_LOGD(TAG, "Livolo transmitting Remote: %d, Key: %d)", remote_id, key_id);

  uint32_t command = ((uint32_t) key_id & 0x7F) | (remote_id << 7);
  livolotx_send_command(command, 23);
}

