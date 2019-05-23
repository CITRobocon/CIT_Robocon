/*
 * odometry.c
 *
 *  Created on: 2019/01/20
 *      Author: Sano
 */

#include "odometry.h"

#include "stm32f4xx_hal.h"
#include "math.h"
#include "math_operations.h"
#include "body.h"
#include "basic_operations.h"

volatile double _odometry_x = 0.0;
volatile double _odometry_y = 0.0;
volatile double _odometry_angle = 0.0;
volatile double _odometry_av_rightwheel = 0.0;
volatile double _odometry_av_leftwheel = 0.0;

double odometry_get_x (void){
	return _odometry_x;
}

double odometry_get_y (void){
	return _odometry_y;
}

double odometry_get_angle (void){
	return _odometry_angle;
}

double odometry_get_av_rightwheel (void){
	return _odometry_av_rightwheel;
}

double odometry_get_av_leftwheel (void){
	return _odometry_av_leftwheel;
}

void odometry_set_position (double x, double y, double angle){
	_odometry_x = x;
	_odometry_y = y;
	_odometry_angle = angle;
}

void odometry_update (void){
	const double ppr = 4000, period = 0.010;

	volatile short d_enc_r, d_enc_l;
	volatile double d_angle;

	d_enc_r = enc_right_getAndResetCount();
	d_enc_l = enc_left_getAndResetCount();

	_odometry_av_rightwheel = (-(double)(d_enc_r-d_enc_l)/ENCODER_DISTANCE*(ENCODER_DISTANCE-WHEEL_DISTANCE)/2.0 + (double)d_enc_r)/ppr*2.0*PI*ENCODER_RADIUS/WHEEL_RADIUS/period;
	_odometry_av_leftwheel = ((double)(d_enc_r-d_enc_l)/ENCODER_DISTANCE*(ENCODER_DISTANCE-WHEEL_DISTANCE)/2.0 + (double)d_enc_l)/ppr*2.0*PI*ENCODER_RADIUS/WHEEL_RADIUS/period;

	d_angle = (double)(d_enc_r-d_enc_l)/ppr*2.0*PI*ENCODER_RADIUS/ENCODER_DISTANCE;

	_odometry_x += (double)(d_enc_r+d_enc_l)/2.0/ppr*2.0*PI*ENCODER_RADIUS*cos(_odometry_angle+d_angle/2.0)*sinc(d_angle/2.0);
	_odometry_y += (double)(d_enc_r+d_enc_l)/2.0/ppr*2.0*PI*ENCODER_RADIUS*sin(_odometry_angle+d_angle/2.0)*sinc(d_angle/2.0);

	_odometry_angle += d_angle;
	if (_odometry_angle > PI)
		_odometry_angle -= 2.0*PI;
	else if (_odometry_angle < -PI)
		_odometry_angle += 2.0*PI;
}
