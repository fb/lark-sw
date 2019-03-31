#include "esp_i2c.h"
#include "esp_pins.h"

#include "driver/i2c.h"

#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_MASTER_FREQ_HZ         100000

#define I2C_TIMEOUT 10
#define TIMESLICES(x) ((x+portTICK_RATE_MS-1)/portTICK_RATE_MS)

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

int i2c_master_port;

#define MUX_ADDR 0x70

void i2c_init(void)
{
	i2c_master_port = I2C_NUM_0;
	i2c_config_t i2c_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = 33,
		.scl_io_num = 32,
		.master = {
			.clk_speed = I2C_MASTER_FREQ_HZ
		}
	};

	i2c_param_config(i2c_master_port, &i2c_conf);
	i2c_driver_install(i2c_master_port, i2c_conf.mode,
			I2C_MASTER_RX_BUF_DISABLE,
			I2C_MASTER_TX_BUF_DISABLE, 0);

    i2c_write_byte(MUX_ADDR, 0x06);
    uint8_t buf[2];
    i2c_read_bytes(MUX_ADDR, 1, buf);
    printf("mux: %02X\n", buf[0]);

    i2c_write_byte(0x77, 0xA2);
    i2c_read_bytes(0x77, 2, buf);
    printf("u: %u\n", buf[0] << 8 | buf[1]);
}

esp_err_t i2c_read_bytes(uint8_t addr, size_t length, uint8_t * data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | READ_BIT, ACK_CHECK_EN);
	if(length > 1)
		i2c_master_read(cmd, data, length - 1, ACK_VAL);
    i2c_master_read_byte(cmd, data + length - 1, NACK_VAL);
	i2c_master_stop(cmd);
	int ret = i2c_master_cmd_begin(i2c_master_port, cmd, TIMESLICES(I2C_TIMEOUT));
	i2c_cmd_link_delete(cmd);
	return ret;
}

esp_err_t i2c_write_bytes(uint8_t addr, size_t length, uint8_t * data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data, length, ACK_VAL);
	i2c_master_stop(cmd);
	int ret = i2c_master_cmd_begin(i2c_master_port, cmd, TIMESLICES(I2C_TIMEOUT));
	i2c_cmd_link_delete(cmd);
	return ret;
}

esp_err_t i2c_write_byte(uint8_t addr, uint8_t data)
{
    return i2c_write_bytes(addr, 1, &data);
}
