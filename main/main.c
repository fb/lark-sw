/* Lark main
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 *
 * This file is part of Lark.
 *
 * Lark is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lark is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Lark.  If not, see <http://www.gnu.org/licenses/>.
 */



#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

#include "audiovario.h"
#include "controls.h"
#include "net.h"
#include "semaphores.h"
#include "sensor.h"
#include "vario.h"




/* globals */
#define STACK_SIZE 4096


#define BLINK_GPIO 5
void debug_task(void *pvParameter)
{
    //gpio_pad_select_gpio(BLINK_GPIO); //default?
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while(1) {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


void app_main(void ) {
    	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	xTaskCreate(&debug_task, "debug_task", STACK_SIZE, NULL, 4, NULL);

	net_feed_semaphore = xSemaphoreCreateBinary();
	audio_feed_semaphore = xSemaphoreCreateBinary();

	printf("Running 1\n");
	sensor_read_init();
	printf("Running 2\n");
	//audiovario_start();

	xTaskCreate(&networking_task, "networking_task", STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(&control_task, "control_task", STACK_SIZE, NULL, 4, NULL);
}



