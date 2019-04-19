/* Lark TE vario computer
 * Copyright (C) 2019 Fabian Bartschke
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
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
//#include "freertos/event_groups.h"

#include "semaphores.h"
#include "sensor.h"
#include "nmea.h"

#define STACK_SIZE 4096

static void nmea_mux_task(void *pvParameter) {
	/* run main loop */
	while(1) {
		if (xSemaphoreTake(sensor_event_semaphore, portMAX_DELAY)!= pdTRUE) {}
            //ESP_LOGW(TAG, "semaphore failed!\n");

        char sentence[256];
        extern sensor_event_t sensor_event;
        if(sensor_event.type == EV_Pstat)
        {
                POV_sentence_float(sentence, 'P', sensor_event.value);
                printf("%s\n", sentence);
        }
    }
}

int nmea_mux_init(void) {
	/* create read task */
	xTaskCreate(&nmea_mux_task, "NEMA mux", STACK_SIZE, NULL, 6, NULL);

	return ESP_OK;
}

