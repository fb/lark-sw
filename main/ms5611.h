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

#ifndef MS5611_H_
#define MS5611_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>


bool ms5611_crc_check (uint16_t *n_prom); // this is from the original driver

/* #ifdef TEST
uint32_t calculate_P(uint32_t D1, uint32_t D2, uint16_t * C, uint8_t * Q);
#
float calculate_P_float(uint32_t D1, int32_t dT, uint16_t * C, uint8_t * Q);
int32_t calculate_dT(uint32_t D2, uint16_t * C, uint8_t * Q);
#endif */

typedef enum
{
    MS_TYPE_BARO, //!< MS5611
    MS_TYPE_DIFF, //!< MS5525
} ms_type_t;

typedef enum
{
    INIT,
    POLL_D1,
    POLL_D2,
} ms_state_t;

typedef struct
{
    ms_state_t state;
    ms_type_t type;
    int channel;
    uint16_t C[8];
    uint8_t * Q;
    uint32_t D1, D2;
    float value;
} sensor_t;

void sensor_init(sensor_t *);
bool sensor_run(sensor_t *);
bool sensor_valid(sensor_t *);

#endif /* MS5611_H_ */
