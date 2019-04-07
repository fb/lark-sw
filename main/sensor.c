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
#include "esp_i2c.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#define STACK_SIZE 4096
#define TAG "sensors: "

#define CONVERTERSION_BEAT_US 100000

/* Sensor device structs */
SemaphoreHandle_t timer_semaphore = NULL;

press_temp_t tep_sensor;

ms_sensor_t sensor1 = {
    .addr = MS_ADDR_76,
    .type = TYPE_MS5611,
};
ms_sensor_t sensor3 = {
    .addr = MS_ADDR_76,
    .type = TYPE_MS5525DS001,
};

static void sensor_read_timer_callback(void *arg) {
	xSemaphoreGive(timer_semaphore);
}

static void sensor_read_task(void *pvParameter) {
	/* run main loop */
	while(1) {
		if (xSemaphoreTake(timer_semaphore, portMAX_DELAY)!= pdTRUE)
            ESP_LOGW(TAG, "semaphore failed!\n");
    }

}

enum
{
//    MS_ADDR_76          = 0x76,
    MS_CMD_RESET        = 0x1E,
    MS_CMD_READ_ADC     = 0x00,
    MS_CMD_CONVERT_D1   = 0x48,
    MS_CMD_CONVERT_D2   = 0x58,
    MS_CMD_READ_PROM_A0 = 0xA0,
};

static uint32_t read_adc()
{
    i2c_write_byte(MS_ADDR_76, MS_CMD_READ_ADC);
    uint8_t buf[3];
    i2c_read_bytes(MS_ADDR_76, 3, buf);
    return buf[2] | buf[1] << 8 | buf[0] << 16;
}

static void read_coeffs(uint16_t C[8])
{
    for(int i = 0; i < 8; i++)
    {
        uint8_t buf[2];
        i2c_write_byte(MS_ADDR_76, MS_CMD_READ_PROM_A0 + 2*i);
        i2c_read_bytes(MS_ADDR_76, 2, buf);

        C[i] = buf[1] | (buf[0] << 8);

    }
}


int sensor_read_init(void) {
	vario_init();

    const uint8_t a_mux = 0x70;

    for(int channel = 0; channel < 1; channel++)
    {
        uint8_t buf[4];

        i2c_write_byte(a_mux, 4 + channel);
        i2c_read_bytes(a_mux, 1, buf);

        uint16_t C[8];
        read_coeffs(C);
        printf("# CRC check: %d\n", ms5611_crc_check(C) == true);

        for(int i = 0; i < 64; i++)
        {
            i2c_write_byte(MS_ADDR_76, MS_CMD_CONVERT_D1); // pressure @ OSR 4096

            vTaskDelay(20 / portTICK_PERIOD_MS);

            uint32_t D1, D2;

            D1 = read_adc();

            i2c_write_byte(MS_ADDR_76, MS_CMD_CONVERT_D2); // temp @ OSR 4096
            vTaskDelay(20 / portTICK_PERIOD_MS);

            D2 = read_adc();

            int32_t dT = calculate_dT(D2, C);
            int32_t P = calculate_P(D1, dT, C);
            printf("%d %u %u %d %d\n", channel, D1, D2, dT, P);
        }
        printf("\n");
    }

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
	esp_timer_start_periodic(read_timer, CONVERTERSION_BEAT_US);

	return ESP_OK;
}

