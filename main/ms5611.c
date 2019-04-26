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

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

// Q exponents (see MS5525/MS5611 datasheet)
// leaving Q[0] empty allows using same indices as datasheet
// MS5611
static uint8_t Q_MS5611[7] = {
    [1] = 15,
    [2] = 16,
    [3] = 8,
    [4] = 7,
    [5] = 8,
    [6] = 23,
};

// MS5525
static uint8_t Q_MS5525[7] =
{
   [1] = 15,
   [2] = 17,
   [3] = 7,
   [4] = 5,
   [5] = 7,
   [6] = 21,
};

int32_t calculate_dT(uint32_t D2, uint16_t * C, uint8_t * Q)
{
    // dT = D2 - C5 * 2**Q5
    int32_t dT = - (C[5] << Q[5]);
    return dT + D2;
}

int64_t calculate_P_noshift(uint32_t D1, uint32_t D2, uint16_t * C, uint8_t * Q)
{
    uint32_t dT =  calculate_dT(D2, C, Q);

    // OFF = C2 * 2**Q2 + (C4 * dT) / 2**Q4
    int64_t OFF = (int64_t)C[2] << 16;
    OFF += (C[4] * dT) >> 7;

    // SENS = C1 * 2**Q1 + (C3 * dT) / 2**Q3
    int64_t SENS = (int64_t)C[1] << Q[1];
    SENS += (C[3] * dT) >> Q[3];

    // P = (D1 * SENS / 2**21 - OFF) / 2**15
    int64_t P = (D1 * SENS) >> 21;
    P -= OFF;
    return P;
}

uint32_t calculate_P(uint32_t D1, uint32_t D2, uint16_t * C, uint8_t * Q)
{
    return calculate_P_noshift(D1, D2, C, Q) >> 15;
}

float calculate_P_float(uint32_t D1, uint32_t D2, uint16_t * C, uint8_t * Q)
{
    return calculate_P_noshift(D1, D2, C, Q) * (0.01 / (1 << 15));
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

enum ms_addr
{
    MS_ADDR_76 = 0x76,
    MS_ADDR_77 = 0x77,
};

enum ms_cmd
{
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

void sensor_init(sensor_t * sensor)
{
    if(sensor->type == MS_TYPE_DIFF)
	sensor->Q = Q_MS5525;
    else
	sensor->Q = Q_MS5611;

    sensor->state = INIT;
}

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
            int32_t P = calculate_P(sensor->D1, sensor->D2, sensor->C, sensor->Q);
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

#ifdef __cplusplus
}
#endif
