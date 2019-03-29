#include "esp_i2c.h"
#include "esp_pins.h"

#include "driver/i2c.h"

#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_MASTER_FREQ_HZ         100000

void i2c_init(void)
{
	/* I2C for sensors */
	int i2c_master_port = I2C_NUM_0;
	i2c_config_t i2s_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GPIO_I2C_SDA,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = GPIO_I2C_SCL,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master = {
			.clk_speed = I2C_MASTER_FREQ_HZ
		}
	};

	i2c_param_config(i2c_master_port, &i2s_conf);
	i2c_driver_install(i2c_master_port, i2s_conf.mode,
			I2C_MASTER_RX_BUF_DISABLE,
			I2C_MASTER_TX_BUF_DISABLE, 0);
}
