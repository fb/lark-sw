/* Lark TE vario computer
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 * Copyright (C) 2014  The openvario project
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

#include <math.h>

#include "sensor.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#define TAG "vario: "

float vario_val;

/* Kalman filter implementation taken from OpenVario. */
typedef struct {
        float x_abs_;           // the absolute quantity x
        float x_vel_;           // the rate of change of x, in x units per second squared.
        
        // Covariance matrix for the state
        float p_abs_abs_;
        float p_abs_vel_;
        float p_vel_vel_;
        
        // The variance of the acceleration noise input to the system model
        float var_x_accel_;
} t_kalmanfilter1d;


static void KalmanFiler1d_update(t_kalmanfilter1d* filter, float z_abs, float var_z_abs, float dt)
{
	float F1=1.0;
	float dt2, dt3, dt4;
	float y;
	float s_inv;
	float k_abs;
	float k_vel;
	
	// check if dt is positive
	
	//Predict step
	//update state estimate
	filter->x_abs_ += filter->x_vel_ * dt;
	
	// update state covariance
	dt2 = dt * dt;
	dt3 = dt * dt2;
	dt4 = dt2 * dt2;

	filter->p_abs_abs_ += 2*(dt*filter->p_abs_vel_) + dt2 * filter->p_vel_vel_ + 0.25*(filter->var_x_accel_ * dt4);
	filter->p_abs_vel_ += dt * filter->p_vel_vel_ + (filter->var_x_accel_ * dt3)/2;
	filter->p_vel_vel_ += filter->var_x_accel_ * dt2;
	
	// Update step
	y = z_abs - filter->x_abs_;		// Innovation
	s_inv = F1 / (filter->p_abs_abs_ + var_z_abs);		// Innovation precision 
	k_abs = filter->p_abs_abs_*s_inv; // Kalman gain
	k_vel = filter->p_abs_vel_*s_inv;
	
	// Update state estimate.
	filter->x_abs_ += k_abs * y;
	filter->x_vel_ += k_vel * y;
  
	// Update state covariance.
	filter->p_vel_vel_ -= filter->p_abs_vel_ * k_vel;
	filter->p_abs_vel_ -= filter->p_abs_vel_ * k_abs;
	filter->p_abs_abs_ -= filter->p_abs_abs_ * k_abs;

    ESP_LOGD(TAG, "%f %f %f", z_abs, filter->x_abs_, filter->x_vel_);
}

static void KalmanFilter1d_reset(t_kalmanfilter1d* filter)
{
	filter->x_abs_ = 0.0;
	filter->x_vel_ = 0.0;

	filter->p_abs_abs_ = 950.0;
	filter->p_abs_vel_ = 0.0;
	filter->p_vel_vel_ = 0.0;
	
	filter->var_x_accel_ = 0.3;
}

/* TE vario computation from Kalman filter output. Taken from OpenVario. */
static float ComputeVario(const float p, const float d_p)
{
  static const float FACTOR = -2260.389548275485;
  static const float EXP = -0.8097374740609689;
  return FACTOR*pow(p, EXP) * d_p;
}

t_kalmanfilter1d vkf;

void vario_init()
{
    KalmanFilter1d_reset(&vkf);
}

void vario_update(float pte_hectopascal, float var, float dt)
{
    KalmanFiler1d_update(&vkf, pte_hectopascal, var, dt);
}

float vario_get_d_te()
{
    return ComputeVario(vkf.x_abs_, vkf.x_vel_);
}
