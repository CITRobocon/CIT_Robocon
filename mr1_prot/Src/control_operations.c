/*
 * control_operations.c
 *
 *  Created on: 2019/01/21
 *      Author: Sano
 */

#include "math.h"
#include "math_operations.h"
#include "basic_operations.h"
#include "body.h"
#include "odometry.h"


volatile unsigned char _control_state = 0;

volatile int _control_with_shagai = 0;

volatile double _control_t_av_wheel_r = 0.0, _control_t_av_wheel_l = 0.0, _control_av_wheel_mnt = 0.0;
volatile double _control_av_wheel_r_ep = 0.0, _control_av_wheel_l_ep = 0.0;
volatile double _control_av_wheel_r_ei = 0.0, _control_av_wheel_l_ei = 0.0;

volatile vec4 _control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y;
volatile double _control_cubicCurve_vel[5] = {};
volatile int _control_cubicCurve_dir = 1;

volatile double _control_t_av_throwingArm = 0.0;
volatile double _control_av_throwingArm_ep = 0.0;
volatile double _control_av_throwingArm_ei = 0.0;

volatile double _control_t_angle_throwingArm;

unsigned char control_av_wheel_start (double avr, double avl, double mnt, int mode){
	_control_t_av_wheel_r = avr;
	_control_t_av_wheel_l = avl;
	_control_av_wheel_mnt = mnt;
	_control_with_shagai = mode;

	_control_av_wheel_r_ep = 0.0;
	_control_av_wheel_l_ep = 0.0;
	_control_av_wheel_r_ei = 0.0;
	_control_av_wheel_l_ei = 0.0;

	return (_control_state |= 0x01);
}

unsigned char control_follow_cubicCurve_start (vec4 coes_x, vec4 coes_y, double v0, double v1, double v2, double v3, double v4, int mode){
	_control_t_cubicCurve_coes_x = coes_x;
	_control_t_cubicCurve_coes_y = coes_y;

	_control_cubicCurve_vel[0] = v0;
	_control_cubicCurve_vel[1] = v1;
	_control_cubicCurve_vel[2] = v2;
	_control_cubicCurve_vel[3] = v3;
	_control_cubicCurve_vel[4] = v4;

	_control_with_shagai = mode;

	return (_control_state |= 0x02);
}

unsigned char control_av_throwingArm_start (double av, int mode){
	_control_t_av_throwingArm = av;
	_control_with_shagai = mode;

	return (_control_state |= 0x04);
}

unsigned char control_angle_throwingArm_start (double angle, int mode){
	_control_t_angle_throwingArm = angle;
    _control_with_shagai = mode;

	return (_control_state |= 0x08);
}

void control_follow_cubicCurve_invert (void){
	_control_cubicCurve_dir *= -1;
}

unsigned char control_av_wheel_end (void){
	motor3_write(0.0);
	motor4_write(0.0);

	if ((_control_state & 0x01) == 0x01)
		return (_control_state ^= 0x01);
	else
		return _control_state;
}

unsigned char control_follow_cubicCurve_end (void){
	motor3_write(0.0);
	motor4_write(0.0);
	if ((_control_state & 0x02) == 0x02)
		return (_control_state ^= 0x02);
	else
		return _control_state;
}

unsigned char control_av_throwingArm_end (void){
	_control_av_throwingArm_ep = 0.0;
	_control_av_throwingArm_ei = 0.0;

	motor2_write(0.0);
	if ((_control_state & 0x04) == 0x04)
		return (_control_state ^= 0x04);
	else
		return _control_state;
}

unsigned char control_angle_throwingArm_end (void){
	control_av_throwingArm_end();
	if ((_control_state & 0x08) == 0x08)
		return (_control_state ^= 0x08);
	else
		return _control_state;
}

unsigned char control_get_state (void){
	return _control_state;
}

int isUnderControl (int n){
	return ((_control_state>>(n-1))&0x01);
}

void control_av_wheel (void){
	double epgain = 0.050, eigain = 0.000010, edgain = 0.000010, period = 0.020;
	if (_control_with_shagai){
		epgain = 0.030;
		eigain = 0.000010;
		edgain = 0.000030;
	}

	double ep_r, ep_l;
	ep_r = (_control_t_av_wheel_r - odometry_get_av_rightwheel());
	ep_l = (_control_t_av_wheel_l - odometry_get_av_leftwheel());

	double ed_r, ed_l;
	ed_r = (_control_av_wheel_r_ep - ep_r)/period;
	ed_l = (_control_av_wheel_l_ep - ep_l)/period;

	_control_av_wheel_r_ep = ep_r;
	_control_av_wheel_l_ep = ep_l;

	_control_av_wheel_r_ei += ep_r*period;
	_control_av_wheel_l_ei += ep_l*period;

	motor3_write(epgain*_control_av_wheel_r_ep + eigain*_control_av_wheel_r_ei + edgain*ed_r + _control_av_wheel_mnt/WHEEL_DISTANCE/MOTOR_WHEEL_MAX_T*WHEEL_RADIUS);
	motor4_write(epgain*_control_av_wheel_l_ep + eigain*_control_av_wheel_l_ei + edgain*ed_l - _control_av_wheel_mnt/WHEEL_DISTANCE/MOTOR_WHEEL_MAX_T*WHEEL_RADIUS);
}

void control_follow_cubicCurve (void){
	static double u = 0.0;
	static int vel_grad = 0;

	const double v = (_control_cubicCurve_vel[vel_grad]+(_control_cubicCurve_vel[vel_grad+1]-_control_cubicCurve_vel[vel_grad])*(4.0*u-vel_grad));
	double fbgain_phi = 1.0, fbgain_y, mnt;
	vec2 cp, t_cp;
	cp = num2vec2(odometry_get_x(), odometry_get_y());

  	if (v == 0.0)
    	fbgain_y = 0.0;
    else
	    fbgain_y = -8.0/v;

  	if (_control_with_shagai){
  		fbgain_phi *= 0.8;
  		fbgain_y *= 0.6;
  	}

	u = cubicCurve_get_nearVar(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, cp, u);
	if (u < 0.0)
		u = 0.0;

	if (4.0*u-vel_grad > 1.0)
		vel_grad++;

	t_cp = cubicCurve_get_point(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, u);

	volatile double d_phi = odometry_get_angle() - cubicCurve_get_vecAngle(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, u);
	volatile double d_y = (t_cp.v1-cp.v1)*sin(-odometry_get_angle()) + (t_cp.v2-cp.v2)*cos(-odometry_get_angle());

	if (_control_cubicCurve_dir == 1)
		d_phi = odometry_get_angle() - cubicCurve_get_vecAngle(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, u);
	else if (odometry_get_angle() < 0.0)
		d_phi = -(odometry_get_angle()+PI - cubicCurve_get_vecAngle(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, u));
	else
		d_phi = -(odometry_get_angle()-PI - cubicCurve_get_vecAngle(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, u));

	if (d_phi < -PI)
    	d_phi += 2.0*PI;
	else if (d_phi > PI)
    	d_phi -= 2.0*PI;

    volatile double ff, curve_radius = cubicCurve_get_curvRad(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, u);
    if (isnan(curve_radius)){
    	ff = 0.0;
    	mnt = 0.0;
    }else{
    	ff = _control_cubicCurve_dir*v/curve_radius*WHEEL_DISTANCE/2.0;
    	if (_control_with_shagai)
    		mnt = M_THROWING_ARM_WITH_SHAGAI*v*v/curve_radius*L_SG_THROWING_ARM_WITH_SHAGAI;
    	else
    		mnt = 0.0;
    }

    if (u < 0.97 && (u < 0.5 || odometry_moving()))
    	control_av_wheel_start (_control_cubicCurve_dir*(v+ff-fbgain_phi*d_phi-fbgain_y*d_y)/WHEEL_RADIUS, _control_cubicCurve_dir*(v-ff+fbgain_phi*d_phi+fbgain_y*d_y)/WHEEL_RADIUS, -mnt, _control_with_shagai);
    else{
		led3c_write(1,0);
    	control_av_wheel_end();
    	control_follow_cubicCurve_end();
    	u = 0.0;
    	vel_grad = 0;
    }
}

void control_av_throwingArm (void){
	const double epgain = 0.050, eigain = 0.80, edgain = 0.0, period = 0.020;
	double linearizegain;

	if (_control_with_shagai)
		linearizegain = (M_THROWING_ARM_WITH_SHAGAI*G*L_SG_THROWING_ARM_WITH_SHAGAI/I_THROWING_ARM_WITH_SHAGAI/MOTOR_THROWING_ARM_MAX_T);
	else
		linearizegain = (M_THROWING_ARM*G*L_SG_THROWING_ARM/I_THROWING_ARM/MOTOR_THROWING_ARM_MAX_T);

	double ep = _control_t_av_throwingArm-enc_arm_getAV();

	double ed = (_control_av_throwingArm_ep-ep)/period;
	_control_av_throwingArm_ep = ep;
	_control_av_throwingArm_ei += ep*period;

	motor2_write(epgain*_control_av_throwingArm_ep + eigain*_control_av_throwingArm_ei + edgain*ed + linearizegain*cos(enc_arm_getAngle_rad()));
}

void control_angle_throwingArm (void){
	const double fbgain = 1.5;

	control_av_throwingArm_start(fbgain*(_control_t_angle_throwingArm-enc_arm_getAngle_rad()), 0);
}
