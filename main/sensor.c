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
#include "filter.h"
#include "ms5611.h"
#include "sensor.h"
#include "vario.h" // vario_feed
#include "esp_i2c.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#define STACK_SIZE 4096
#define TAG "sensors: "

#define CONVERTERSION_BEAT_US 10000

SemaphoreHandle_t timer_semaphore = NULL;
SemaphoreHandle_t sensor_event_semaphore = NULL;

sensor_event_t sensor_event;

static void sensor_read_timer_callback(void *arg) {
	xSemaphoreGive(timer_semaphore);
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

typedef enum
{
    INIT,
    POLL_D1,
    POLL_D2,
} sensor_state_t;

typedef struct
{
    sensor_state_t state;
    int channel;
    uint16_t C[8];
    uint32_t D1, D2;
    float value;
} sensor_t;

// Sensor FSM
bool sensor_run(sensor_t * sensor)
{
    i2c_write_byte(0x70, 4 + sensor->channel); // activate my channel on MUX

    switch(sensor->state)
    {
        case INIT:
            read_coeffs(sensor->C);
            printf("# CRC check: %d\n", ms5611_crc_check(sensor->C) == true);
            sensor->state = POLL_D1;
        case POLL_D2:
            sensor->D1 = read_adc();
            i2c_write_byte(MS_ADDR_76, MS_CMD_CONVERT_D2); // temperature @ OSR 4096
            int32_t dT = calculate_dT(sensor->D2, sensor->C);
            int32_t P = calculate_P(sensor->D1, dT, sensor->C);
            //printf("%d %u %u %d %d\n", sensor->channel, sensor->D1, sensor->D2, dT, P);
            printf("%d %d\n", sensor->channel, P);
            sensor->state = POLL_D1;
            sensor->value = P / 100.0f;
            return true;
            break;
        case POLL_D1:
            sensor->D2 = read_adc();
            i2c_write_byte(MS_ADDR_76, MS_CMD_CONVERT_D1); // pressure @ OSR 4096
            sensor->state = POLL_D2;
            break;
        default:
            printf("FSM error\n");
    }

    return false;
}

bool sensor_valid(sensor_t * sensor)
{
    return sensor->value > 200 && sensor->value < 1200;
}

static void sensor_read_task(void *pvParameter) {
	/* run main loop */
	static exp_filter_t p1_filter;
	exp_filter_init(&p1_filter, 0.8);

	while(1) {
        static sensor_t sensor1 =
        {
            .state = INIT,
            .channel = 0,
        };

        static sensor_t sensor2 =
        {
            .state = INIT,
            .channel = 1,
        };

        static sensor_t sensor3 =
        {
            .state = INIT,
            .channel = 2,
        };

		if (xSemaphoreTake(timer_semaphore, portMAX_DELAY)!= pdTRUE)
            ESP_LOGW(TAG, "semaphore failed!\n");

        /* run sensor FSMs */
        bool new_s1 = sensor_run(&sensor1);
        bool new_s2 = sensor_run(&sensor2);
        sensor_run(&sensor3);

        /* filter sensor data */

        static int p1_count = 0;
        if(new_s1 == true && sensor_valid(&sensor1)) // new p1 data available
	{
	    /* exponential smoothing */
	    exp_filter_feed(&p1_filter, sensor1.value);
            p1_count--;
            if(p1_count <= 0)
            {
                p1_count = 25; // this translates to 50 Hz / 25 = 2 Hz

                // publish the filtered value
                sensor_event.type = EV_Pstat;
                sensor_event.value = exp_filter_get(&p1_filter);
                xSemaphoreGive(sensor_event_semaphore);
            }
        }

        static int p2_count = 5;
        if(new_s2 == true && sensor_valid(&sensor2))
        {
            vario_update(sensor2.value, 1 / 50.0);
            p2_count--;

            if(p2_count <= 0)
            {
                p2_count = 25;

                sensor_event.type = EV_Pte;
                sensor_event.value = vario_get_tek();
                xSemaphoreGive(sensor_event_semaphore);
            }
        }
    }
}

int sensor_read_init(void) {
	vario_init();

	/* create read semaphore */
	timer_semaphore = xSemaphoreCreateBinary();
	sensor_event_semaphore = xSemaphoreCreateBinary();

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

