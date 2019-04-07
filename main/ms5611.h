/**
 * \file ms5611.h
 *
 * \brief ms5611 Temperature sensor driver header file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 *
 * \asf_license_stop
 *
 */

#ifndef ms5611_H_INCLUDED
#define ms5611_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

void mux_set_channel(uint8_t);

enum ms5611_resolution_osr {
	ms5611_resolution_osr_256 = 0,
	ms5611_resolution_osr_512,
	ms5611_resolution_osr_1024,
	ms5611_resolution_osr_2048,
	ms5611_resolution_osr_4096
};

enum ms5611_status {
	ms5611_status_ok,
	ms5611_status_no_i2c_acknowledge,
	ms5611_status_i2c_transfer_error,
	ms5611_status_crc_error
};

enum ms_addr {
    MS_ADDR_76 = 0x76,
    MS_ADDR_77 = 0x77,
};

enum ms_type {
    TYPE_MS5525DS001,
    TYPE_MS5611,
    TYPE_COUNT,
};


typedef struct ms_sensor {
    enum ms_addr addr;
    enum ms5611_resolution_osr ms5611_resolution_osr;
    enum ms_type type;
    uint16_t eeprom_coeff[8];
} ms_sensor_t;


// Functions

/**
 * \brief Configures the SERCOM I2C master to be used with the ms5611 device.
 */
void ms5611_init(ms_sensor_t *); 

/**
 * \brief Check whether ms5611 device is connected
 *
 * \return bool : status of ms5611
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
//bool ms5611_is_connected(void);

/**
 * \brief Reset the ms5611 device
 *
 * \return ms5611_status : status of ms5611
 *       - ms5611_status_ok : I2C transfer completed successfully
 *       - ms5611_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5611_status_no_i2c_acknowledge : I2C did not acknowledge
 */
//enum ms5611_status ms5611_reset(void);

/**
 * \brief Reads the temperature and pressure ADC value and compute the compensated values.
 *
 * \param[out] float* : Celsius Degree temperature value
 * \param[out] float* : mbar pressure value
 *
 * \return ms5611_status : status of ms5611
 *       - ms5611_status_ok : I2C transfer completed successfully
 *       - ms5611_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5611_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5611_status_crc_error : CRC check error on on the PROM coefficients
 */
enum ms5611_status ms5611_read_temperature_and_pressure(ms_sensor_t *, float *, float *);
bool ms5611_crc_check (uint16_t *n_prom);

uint32_t calculate_P(uint32_t D1, int32_t dT, uint16_t * C);
float calculate_P_float(uint32_t D1, int32_t dT, uint16_t * C);
int32_t calculate_dT(uint32_t D2, uint16_t * C);

#endif /* ms5611_H_INCLUDED */
