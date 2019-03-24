#include "esp_i2c.h"
#include "esp_pins.h"

#include "driver/i2c.h"

#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_MASTER_FREQ_HZ         100000

#define I2C_TIMEOUT 1000
#define TIMESLICES(x) ((x+portTICK_RATE_MS-1)/portTICK_RATE_MS)

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

int i2c_master_port;

void i2c_init(void)
{
	/* I2C for sensors */
	i2c_master_port = I2C_NUM_0;
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

int i2c_read_bytes(uint8_t addr, size_t length, uint8_t * data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | 0, ACK_CHECK_EN);
	if(length > 1)
		i2c_master_read(cmd, data, length, ACK_VAL);
	else
		i2c_master_read_byte(cmd, data, NACK_VAL);
	i2c_master_stop(cmd);
	int ret = i2c_master_cmd_begin(i2c_master_port, cmd, TIMESLICES(I2C_TIMEOUT));
	i2c_cmd_link_delete(cmd);
	return ret;
}

