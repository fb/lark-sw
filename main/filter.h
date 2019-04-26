/* Lark generic filters
 * Copyright (C) 2019  The LarkVario project
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

#ifndef FILTER_H_
#define FILTER_H_

typedef struct
{
	float alpha;
	float value;
} exp_filter_t;

static void exp_filter_init(exp_filter_t * f, float alpha)
{
	f->value = 0;
	f->alpha = alpha;
}

static void exp_filter_feed(exp_filter_t * f, float input)
{
	f->value = f->alpha * f->value  + (1 - f->alpha) * input;
}

static float exp_filter_get(exp_filter_t * f)
{
	return f->value;
}

#endif
