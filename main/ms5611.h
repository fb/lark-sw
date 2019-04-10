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

enum ms_addr {
    MS_ADDR_76 = 0x76,
    MS_ADDR_77 = 0x77,
};

bool ms5611_crc_check (uint16_t *n_prom); // this is from the original driver

uint32_t calculate_P(uint32_t D1, int32_t dT, uint16_t * C);
float calculate_P_float(uint32_t D1, int32_t dT, uint16_t * C);
int32_t calculate_dT(uint32_t D2, uint16_t * C);

#endif /* ms5611_H_INCLUDED */
