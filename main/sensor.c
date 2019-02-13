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
#include "freertos/queue.h"
#include "driver/i2c.h"

#include "esp_err.h"
//#include "nvs_flash.h"

//#include "sdkconfig.h"

#include "semaphores.h"
#include "ms5611.h"
#include "sensor.h"
#include "vario.h" // vario_feed

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#define STACK_SIZE 4096
#define TAG "sensors: "


#define I2C_SCL 18
#define I2C_SDA 21
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_TEP_ADDR 0x77

#define CONVERSION_BEAT_US 12500


/* Sensor device structs */
ms5611_drv_t tep_dev;
SemaphoreHandle_t timer_semaphore = NULL;
press_temp_t tep_sensor;

void sensor_read_round()
{
	static int stage = 0;

	static int32_t rawpress = 0;
	static int32_t rawtemp = 0;
    static int32_t last_sample_microseconds = 0; // 32 bits should be enough

    tep_sensor.header = 0x5555;
    tep_sensor.time = (uint16_t)esp_timer_get_time();

	switch (stage) {
		case 0: // at t = 0
			/* TEK pressure */
			rawpress = ms5611_get_conv(&tep_dev);

            ms5611_start_conv_temp(&tep_dev);

            // TODO: why does this happen sometimes?
            if(tep_sensor.press_mbar < 150.0)
                break; // invalid

            int32_t this_sample_microseconds = esp_timer_get_time();
            int32_t diff = (this_sample_microseconds - last_sample_microseconds);
            last_sample_microseconds = this_sample_microseconds;
            float dt_seconds = diff / 1e6;

            vario_update(tep_sensor.press_mbar, 0.25, dt_seconds);

            /* Start temp measurement */
			break;
        case 1: // at t = 12.5 ms
            /* read temp values */
        	rawtemp = ms5611_get_conv(&tep_dev);

        	/* start press */
        	ms5611_start_conv_press(&tep_dev);

        	// convert / display
            tep_sensor.press_mbar = ms5611_get_pressure(&tep_dev, rawpress, rawtemp);
            tep_sensor.temp_celsius = ms5611_get_temp(&tep_dev, rawtemp);
            ESP_LOGD(TAG, "tep: %d %d %2.2f %3.3f %5.2f", rawpress, rawtemp, tep_sensor.temp_celsius, tep_sensor.press_mbar, ms5611_calc_altitude(tep_sensor.press_mbar));
			/* Unblock network -> feed data */
			xSemaphoreGive(net_feed_semaphore);
            break;
		default:
            break;
	}

	if(stage == 2)
		stage = 0;
	else
		stage++;
}

static float compute_pressure(ms5611_drv_t *dev) {
	int32_t rawtemp;
	int32_t rawpress;

	ms5611_start_conv_temp(dev);
	vTaskDelay(20/portTICK_PERIOD_MS);
	rawtemp = ms5611_get_conv(dev);

	ms5611_start_conv_press(dev);
	vTaskDelay(20/portTICK_PERIOD_MS);
	rawpress = ms5611_get_conv(dev);

	return ms5611_get_pressure(dev, rawpress, rawtemp);
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
	/* I2C for sensors */
	int i2c_master_port = I2C_NUM_0;
	i2c_config_t i2s_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_SDA,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = I2C_SCL,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master = {
			.clk_speed = I2C_MASTER_FREQ_HZ
		}
	};

	i2c_param_config(i2c_master_port, &i2s_conf);
	i2c_driver_install(i2c_master_port, i2s_conf.mode,
		I2C_MASTER_RX_BUF_DISABLE,
		I2C_MASTER_TX_BUF_DISABLE, 0);

	int ret = -1;
	while(ret != 0)
	{
		ret = ms5611_init(&tep_dev, I2C_NUM_0, I2C_TEP_ADDR);
		ESP_LOGD(TAG, "%s: ms5611 init: %d\n", __func__, ret);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	float tep_init = 0;
	int sensor_test = 0;
	while ((tep_init < 100) || (tep_init > 1200) || (sensor_test > 1000)) {
		tep_init = compute_pressure(&tep_dev);
		sensor_test++;
	}
	if (sensor_test >= 1000) {
		ESP_LOGE(TAG, "Can not read from TEP sensor. Finish.");
		return ESP_FAIL;
	}

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

