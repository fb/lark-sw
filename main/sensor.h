/* Lark sensor module
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 * Copyright (C) 2018 Fabian Bartschke
 *
 * This file is part of Lark.
 *
 * Lark is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lark is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Lark.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>

typedef struct
{
    uint16_t header;
    uint16_t time;
    float press_mbar;
    float temp_celsius;
} press_temp_t;

int sensor_read_init(void);

#endif
