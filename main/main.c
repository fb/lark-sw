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

#include "esp_pins.h"
#include "esp_board.h"

#include "sdkconfig.h"

#include "audiovario.h"
#include "bluetooth_spp.h"
#include "controls.h"
#include "net.h"
#include "nmea_mux.h"
#include "semaphores.h"
#include "sensor.h"

/* globals */
#define STACK_SIZE 4096


#define BLINK_GPIO GPIO_LED1
void debug_task(void *pvParameter)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while(1) {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


void app_main(void ) {
	board_init();

	xTaskCreate(&debug_task, "debug_task", STACK_SIZE, NULL, 4, NULL);

	net_feed_semaphore = xSemaphoreCreateBinary();
	audio_feed_semaphore = xSemaphoreCreateBinary();

	bluetooth_init();
	xTaskCreate(&networking_task, "networking_task", STACK_SIZE, NULL, 4, NULL);

	printf("Running 1\n");
	sensor_read_init();
	printf("Running 2\n");
    nmea_mux_init();
	//audiovario_start();

	xTaskCreate(&control_task, "control_task", STACK_SIZE, NULL, 4, NULL);
}



