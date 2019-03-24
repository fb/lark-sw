#ifndef ESP_I2C_H_
#define ESP_I2C_H_

#include <stdint.h>
#include <stdlib.h>

void i2c_init(void);

int i2c_read_bytes(uint8_t addr, size_t length, uint8_t * data);
int i2c_write_bytes(uint8_t addr, size_t length, uint8_t * data);

#endif // ESP_I2C_H_
