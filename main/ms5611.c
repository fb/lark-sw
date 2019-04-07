/**
 * \file ms5611.c
 *
 * \brief MS5611 Temperature sensor driver source file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to ms5611 datasheet :
 * http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
 *
 */

#include "ms5611.h"

#include "esp_i2c.h"
#include "esp_delay.h"

#include <stdio.h>


#define MUX_ADDR 0x70

void mux_set_channel(uint8_t ch)
{
    if(ch > 3)
        return;

    i2c_write_byte(MUX_ADDR, 4 + ch);
    uint8_t buf;
    i2c_read_bytes(MUX_ADDR, 0, &buf);
}


/**
 * The header "i2c.h" has to be implemented for your own platform to 
 * conform the following protocol :
 */
enum i2c_transfer_direction {
	I2C_TRANSFER_WRITE = 0,
	I2C_TRANSFER_READ  = 1,
};

enum status_code {
	STATUS_OK           = 0x00,
	STATUS_ERR_OVERFLOW = 0x01,
	STATUS_ERR_TIMEOUT  = 0x02,
};

struct i2c_master_packet {
	// Address to slave device
	uint8_t address;
	// Length of data array
	size_t data_length;
	// Data array containing all data to be transferred
	uint8_t *data;
};

enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *const packet)
{
	int ret = i2c_read_bytes((uint8_t)packet->address, (size_t)packet->data_length, packet->data);
	if(ret != 0)
		return STATUS_ERR_TIMEOUT;

	return STATUS_OK;
}

enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *const packet)
{
	int ret = i2c_write_bytes((uint8_t)packet->address, (size_t)packet->data_length, packet->data);
	if(ret != 0)
		return STATUS_ERR_TIMEOUT;

	return STATUS_OK;
}

void delay_ms(uint32_t time_ms)
{
    esp_delay_ms(time_ms);
}

#ifdef __cplusplus
extern "C" {
#endif

// Constants

// MS5611 device commands
#define MS5611_RESET_COMMAND										0x1E
#define MS5611_START_PRESSURE_ADC_CONVERSION						0x40
#define MS5611_START_TEMPERATURE_ADC_CONVERSION						0x50
#define MS5611_READ_ADC												0x00

#define MS5611_CONVERSION_OSR_MASK									0x0F

#define MS5611_CONVERSION_TIME_OSR_256								1000
#define MS5611_CONVERSION_TIME_OSR_512								2000
#define MS5611_CONVERSION_TIME_OSR_1024								3000
#define MS5611_CONVERSION_TIME_OSR_2048								5000
#define MS5611_CONVERSION_TIME_OSR_4096								9000

// MS5611 commands
#define MS5611_PROM_ADDRESS_READ_ADDRESS_0							0xA0
#define MS5611_PROM_ADDRESS_READ_ADDRESS_1							0xA2
#define MS5611_PROM_ADDRESS_READ_ADDRESS_2							0xA4
#define MS5611_PROM_ADDRESS_READ_ADDRESS_3							0xA6
#define MS5611_PROM_ADDRESS_READ_ADDRESS_4							0xA8
#define MS5611_PROM_ADDRESS_READ_ADDRESS_5							0xAA
#define MS5611_PROM_ADDRESS_READ_ADDRESS_6							0xAC
#define MS5611_PROM_ADDRESS_READ_ADDRESS_7							0xAE

// Coefficients indexes for temperature and pressure computation
#define MS5611_CRC_INDEX	7
#define MS5611_C1			1
#define MS5611_C2			2
#define MS5611_C3			3
#define MS5611_C4			4
#define MS5611_C5			5
#define MS5611_C6			6
#define MS5611_COEFFICIENT_NUMBERS  8

// Static functions
static enum ms5611_status ms5611_write_command(ms_sensor_t * sensor, uint8_t);
static enum ms5611_status ms5611_read_eeprom_coeff(ms_sensor_t * sensor, uint8_t, uint16_t*);
static enum ms5611_status ms5611_read_eeprom(ms_sensor_t * sensor);
static enum ms5611_status ms5611_conversion_and_read_adc(ms_sensor_t * sensor_s, uint8_t, uint32_t *);

typedef struct {
    uint8_t Q1;
    uint8_t Q2;
    uint8_t Q3;
    uint8_t Q4;
    uint8_t Q5;
    uint8_t Q6;
} ms_constants_t;

static ms_constants_t constants[] = {
    [TYPE_MS5525DS001] = {
        .Q1 = 15,
        .Q2 = 17,
        .Q3 = 7,
        .Q4 = 5,
        .Q5 = 7,
        .Q6 = 21,
    },
    [TYPE_MS5611]{
        .Q1 = 15,
        .Q2 = 16,
        .Q3 = 8,
        .Q4 = 7,
        .Q5 = 8,
        .Q6 = 23,
    },
};

/**
 * \brief Reset the MS5611 device
 *
 * \return ms5611_status : status of MS5611
 *       - ms5611_status_ok : I2C transfer completed successfully
 *       - ms5611_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5611_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms5611_status  ms5611_reset(ms_sensor_t * sensor)
{
	return ms5611_write_command(sensor, MS5611_RESET_COMMAND);
}

/**
 * \brief Configures the SERCOM I2C master to be used with the MS5611 device.
 */
void ms5611_init(ms_sensor_t * sensor)
{
    ms5611_reset(sensor);
	sensor->ms5611_resolution_osr = ms5611_resolution_osr_4096;
    enum ms5611_status s = ms5611_read_eeprom(sensor);
    if(s != ms5611_status_ok)
        printf("NOK: %d\n", s);
}

/**
 * \brief Check whether MS5611 device is connected
 *
 * \return bool : status of MS5611
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool ms5611_is_connected(ms_sensor_t * sensor)
{
	enum status_code i2c_status;
	
	struct i2c_master_packet transfer = {
		.address     = sensor->addr,
		.data_length = 0,
		.data        = NULL,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status != STATUS_OK)
		return false;
	
	return true;
}
	
/**
 * \brief Writes the MS5611 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return ms5611_status : status of MS5611
 *       - ms5611_status_ok : I2C transfer completed successfully
 *       - ms5611_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5611_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms5611_status ms5611_write_command(ms_sensor_t * sensor, uint8_t cmd)
{
	enum status_code i2c_status;
	uint8_t data[1];
		
	data[0] = cmd;
		
	struct i2c_master_packet transfer = {
		.address     = sensor->addr,
		.data_length = 1,
		.data        = data,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms5611_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms5611_status_i2c_transfer_error;
	
	return ms5611_status_ok;
}

/**
 * \brief Reads the ms5611 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] uint16_t* : Value read in EEPROM
 *
 * \return ms5611_status : status of MS5611
 *       - ms5611_status_ok : I2C transfer completed successfully
 *       - ms5611_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5611_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5611_status_crc_error : CRC check error on the coefficients
 */
enum ms5611_status ms5611_read_eeprom_coeff(ms_sensor_t * sensor, uint8_t command, uint16_t *coeff)
{
	enum ms5611_status status;
	enum status_code i2c_status;
	uint8_t buffer[2];
	
	buffer[0] = 0;
	buffer[1] = 0;

	/* Read data */
	struct i2c_master_packet read_transfer = {
		.address     = sensor->addr,
		.data_length = 2,
		.data        = buffer,
	};
	
	// Send the conversion command
	status = ms5611_write_command(sensor, command);
	if(status != ms5611_status_ok)
		return status;
	
	i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms5611_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms5611_status_i2c_transfer_error;
		
	*coeff = (buffer[0] << 8) | buffer[1];
    
    if (*coeff == 0)
        return ms5611_status_i2c_transfer_error;
	
	return ms5611_status_ok;	
}

/**
 * \brief Reads the ms5611 EEPROM coefficients to store them for computation.
 *
 * \return ms5611_status : status of MS5611
 *       - ms5611_status_ok : I2C transfer completed successfully
 *       - ms5611_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5611_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5611_status_crc_error : CRC check error on the coefficients
 */
enum ms5611_status ms5611_read_eeprom(ms_sensor_t * sensor)
{
	enum ms5611_status status;
	uint8_t i;
	
	for( i=0 ; i< MS5611_COEFFICIENT_NUMBERS ; i++)
	{
		status = ms5611_read_eeprom_coeff(sensor, MS5611_PROM_ADDRESS_READ_ADDRESS_0 + i*2, sensor->eeprom_coeff+i);
		if(status != ms5611_status_ok)
			return status;
	}
    
	if( !ms5611_crc_check( sensor->eeprom_coeff ) )
		return ms5611_status_crc_error;
	
	return ms5611_status_ok;
}

static enum ms5611_status start_conversion(ms_sensor_t * sensor, uint8_t cmd)
{
	return ms5611_write_command(sensor, cmd);
}

static enum ms5611_status read_adc(ms_sensor_t * sensor, uint32_t *adc)
{
	uint8_t buffer[3];
	
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    struct i2c_master_packet read_transfer = {
		.address     = sensor->addr,
		.data_length = 3,
		.data        = buffer,
	};

	// Send the read command
	enum ms5611_status status = ms5611_write_command(sensor, MS5611_READ_ADC);
	if( status != ms5611_status_ok)
		return status;
	
	enum status_code i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return ms5611_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return ms5611_status_i2c_transfer_error;

	*adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

    printf("%u ", *adc);
	
	return status;
}

static enum ms5611_status ms5611_conversion_and_read_adc(ms_sensor_t * sensor, uint8_t cmd, uint32_t *adc)
{
	// delay conversion
	// TODO: try solve this with scheduling
	
    enum ms5611_status status = start_conversion(sensor, cmd);

	if( status != ms5611_status_ok)
		return status;

    delay_ms(15);

    status = read_adc(sensor, adc);

    return status;
}

/**
 * \brief Reads the temperature and pressure ADC value and compute the compensated values.
 *
 * \param[out] float* : Celsius Degree temperature value
 * \param[out] float* : mbar pressure value
 *
 * \return ms5611_status : status of MS5611
 *       - ms5611_status_ok : I2C transfer completed successfully
 *       - ms5611_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5611_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5611_status_crc_error : CRC check error on the coefficients
 */
enum ms5611_status ms5611_read_temperature_and_pressure(ms_sensor_t * sensor, float *temperature, float *pressure)
{
	enum ms5611_status status = ms5611_status_ok;
	uint32_t adc_temperature, adc_pressure;
	int32_t dT, TEMP;
	int64_t OFF, SENS, P; //, T2, OFF2, SENS2;
	uint8_t cmd;
	
	// First read temperature
	cmd = sensor->ms5611_resolution_osr*2;
	cmd |= MS5611_START_TEMPERATURE_ADC_CONVERSION;
	status = ms5611_conversion_and_read_adc(sensor, cmd, &adc_temperature);
	if( status != ms5611_status_ok)
		return status;

	// Now read pressure
	cmd = sensor->ms5611_resolution_osr*2;
	cmd |= MS5611_START_PRESSURE_ADC_CONVERSION;
	status = ms5611_conversion_and_read_adc(sensor, cmd, &adc_pressure);
	if( status != ms5611_status_ok)
		return status;

    printf("\n");

    if (adc_temperature == 0 || adc_pressure == 0)
        return ms5611_status_i2c_transfer_error;

    ms_constants_t * c = &constants[sensor->type];
	// Difference between actual and reference temperature = D2 - Tref
	dT = (int32_t)adc_temperature - ((int32_t)sensor->eeprom_coeff[MS5611_C5] << c->Q5 );
	
	// Actual temperature = 2000 + dT * TEMPSENS
	TEMP = 2000 + ((int64_t)dT * (int64_t)sensor->eeprom_coeff[MS5611_C6] >> c->Q6 ) ;
	
    /*
	// Second order temperature compensation
	if( TEMP < 2000 )
	{
		T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
		OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		
		if( TEMP < -1500 )
		{
			OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
		}
	}
	else
	{
		T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
		OFF2 = 0 ;
		SENS2 = 0 ;
	}
    */
	
	// Offset at actual temperature OFF = OFF_T1 + TCO * dT
	OFF = ( (int64_t)(sensor->eeprom_coeff[MS5611_C2]) << c->Q2 ) + ( ( (int64_t)(sensor->eeprom_coeff[MS5611_C4]) * dT ) >> c->Q4 ) ;
	//sensorOFF -= OFF2 ;
	
	// Sensitivity at actual temperature = SENS_T1 + TCS * dT
	SENS = ( (int64_t)sensor->eeprom_coeff[MS5611_C1] << c->Q1 ) + ( ((int64_t)sensor->eeprom_coeff[MS5611_C3] * dT) >> c->Q3 ) ;
	//SENS -= SENS2 ;
	
	// Temperature compensated pressure = D1 * SENS - OFF
	P = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15 ;
	
	//*temperature = ( (float)TEMP - T2 ) / 100;
	*temperature = ( (float)TEMP ) / 100;

    if(sensor->type == TYPE_MS5525DS001)
    {
        const float diff_press_PSI = P * 0.0001f;
        static const float offset_PSI = 0.0504;
        static const float Pa_per_PSI = 6894.757f; // 1 PSI = 6894.76 Pascals
        *pressure = (diff_press_PSI + offset_PSI) * Pa_per_PSI;
    }
    else
        *pressure = P * 0.01f;
	
	return status;
}

/**
 * \brief CRC check
 *
 * \param[in] uint16_t *: List of EEPROM coefficients
 * \param[in] uint8_t : crc to compare with
 *
 * \return bool : TRUE if CRC is OK, FALSE if KO
 */
bool ms5611_crc_check (uint16_t *n_prom)
{
    uint8_t cnt, n_bit; 
    uint16_t n_rem; 
    uint16_t crc_read;

    n_rem = 0x00;
    crc_read = n_prom[7]; 
    n_prom[7] = (0xFF00 & (n_prom[7])); 
    for (cnt = 0; cnt < 16; cnt++) 
    {
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = (0x000F & (n_rem >> 12)); 
    n_prom[7] = crc_read;
    n_rem ^= 0x00;
        
	return  ( n_rem == (n_prom[7] & 0x000F) );
}

#ifdef __cplusplus
}
#endif
