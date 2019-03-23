/* Lark TE vario computer
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 * Copyright (C) 2014  The openvario project
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
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_err.h"

#include "semaphores.h"
#include "ms5611.h"
#include "sensor.h"
#include "vario.h" // vario_feed

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#define STACK_SIZE 4096
#define TAG "sensors: "


#define I2C_TEP_ADDR 0x77

#define CONVERSION_BEAT_US 12500


/* Sensor device structs */
SemaphoreHandle_t timer_semaphore = NULL;
press_temp_t tep_sensor;

void sensor_read_round()
{
	//ESP_LOGD(TAG, "tep: %d %d %2.2f %3.3f %5.2f", rawpress, rawtemp, tep_sensor.temp_celsius, tep_sensor.press_mbar, ms5611_calc_altitude(tep_sensor.press_mbar));
}

static void sensor_read_timer_callback(void *arg) {
	xSemaphoreGive(timer_semaphore);
}

static void sensor_read_task(void *pvParameter) {
	/* run main loop */
	while(1) {
		if (xSemaphoreTake(timer_semaphore, portMAX_DELAY)!= pdTRUE)
			ESP_LOGW(TAG, "semaphore failed!\n");
		sensor_read_round();
		//sensor_update_output();
	}
}



int sensor_read_init(void) {
	vario_init();

	/* create read semaphore */
	timer_semaphore = xSemaphoreCreateBinary();

	/* create read task */
	xTaskCreate(&sensor_read_task, "sensor_read_task", STACK_SIZE, NULL, 6, NULL);

	esp_timer_handle_t read_timer;
	esp_timer_create_args_t timer_conf = {
		.callback = &sensor_read_timer_callback,
		.dispatch_method = ESP_TIMER_TASK
	};
	esp_timer_create(&timer_conf, &read_timer);
	esp_timer_start_periodic(read_timer, CONVERSION_BEAT_US);

	return ESP_OK;
}

