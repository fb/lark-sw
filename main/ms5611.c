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

#ifdef __cplusplus
extern "C" {
#endif

/*
   [TYPE_MS5525DS001] = {
   .Q1 = 15,
   .Q2 = 17,
   .Q3 = 7,
   .Q4 = 5,
   .Q5 = 7,
   .Q6 = 21,
   },
*/

// Q (powers of 2) for MS5611. (see MS5525 datasheet)
static uint8_t Q[7] = {
    [1] = 15,
    [2] = 16,
    [3] = 8,
    [4] = 7,
    [5] = 8,
    [6] = 23,
};

int32_t calculate_dT(uint32_t D2, uint16_t * C)
{
    // dT = D2 - C5 * 2**Q5
    int32_t dT = - (C[5] << Q[5]);
    return dT + D2;
}

int64_t calculate_P_noshift(uint32_t D1, int32_t dT, uint16_t * C)
{
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

uint32_t calculate_P(uint32_t D1, int32_t dT, uint16_t * C)
{
    return calculate_P_noshift(D1, dT, C) >> 15;
}

float calculate_P_float(uint32_t D1, int32_t dT, uint16_t *C)
{
    return calculate_P_noshift(D1, dT, C) * (0.01 / (1 << 15));
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
