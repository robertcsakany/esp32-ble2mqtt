#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "stat.h"
#include "esp_heap_caps.h"

/* Constants */
static const char *TAG = "STAT";

const esp_timer_create_args_t stat_print_timer_args = {
           .callback = &stat_print,
           .name = "stat-print"
};

esp_timer_handle_t stat_print_timer;

void stat_start(void) {

	    ESP_ERROR_CHECK(esp_timer_create(&stat_print_timer_args, &stat_print_timer));

	    /* Start the timers - 30s */
	    ESP_ERROR_CHECK(esp_timer_start_periodic(stat_print_timer, 5000000));

}

void stat_stop(void) {
    /* Clean up and finish the example */
    ESP_ERROR_CHECK(esp_timer_stop(stat_print_timer));
    ESP_ERROR_CHECK(esp_timer_delete(stat_print_timer));
}

static void stat_print(void* arg) {
/*	size_t memcnt=esp_himem_get_phys_size();
	size_t memfree=esp_himem_get_free_size();
	printf("Himem has %dKiB of memory, %dKiB of which is free. Testing the free memory...\n", (int)memcnt/1024, (int)memfree/1024);
	assert(test_region(memfree, 0xaaaa));

	uint32_t iram =  xPortGetFreeHeapSizeTagged(MALLOC_CAP_32BIT);
	ESP_LOGI(TAG,"free DRAM %u IRAM %u", esp_get_free_heap_size(), iram);
	*/
	ESP_LOGI(TAG,"free DRAM %u", esp_get_free_heap_size());

}
