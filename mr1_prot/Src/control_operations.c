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


extern const double  WHEEL_DISTANCE;
extern const double  WHEEL_RADIUS;
extern const double  ENCODER_DISTANCE;
extern const double  ENCODER_RADIUS;

volatile unsigned char _control_state = 0;

volatile double _control_t_av_wheel_r = 0.0, _control_t_av_wheel_l = 0.0;
volatile double _control_av_wheel_r_ep = 0.0, _control_av_wheel_l_ep = 0.0;
volatile double _control_av_wheel_r_ei = 0.0, _control_av_wheel_l_ei = 0.0;

volatile vec4 _control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y;
volatile double _control_cubicCurve_vel[5] = {};

volatile double _control_t_av_throwingArm = 0.0;
volatile double _control_av_throwingArm_ep = 0.0;
volatile double _control_av_throwingArm_ei = 0.0;

unsigned char control_av_wheel_start (double avr, double avl){
	_control_t_av_wheel_r = avr;
	_control_t_av_wheel_l = avl;
	_control_av_wheel_r_ep = 0.0;
	_control_av_wheel_l_ep = 0.0;
	_control_av_wheel_r_ei = 0.0;
	_control_av_wheel_l_ei = 0.0;

	return (_control_state |= 0x01);
}

unsigned char control_follow_cubicCurve_start (vec4 coes_x, vec4 coes_y, double v0, double v1, double v2, double v3, double v4){
	_control_t_cubicCurve_coes_x = coes_x;
	_control_t_cubicCurve_coes_y = coes_y;

	_control_cubicCurve_vel[0] = v0;
	_control_cubicCurve_vel[1] = v1;
	_control_cubicCurve_vel[2] = v2;
	_control_cubicCurve_vel[3] = v3;
	_control_cubicCurve_vel[4] = v4;

	return (_control_state |= 0x02);
}

unsigned char control_av_throwingArm_start (double av){
	_control_t_av_throwingArm = av;

	return (_control_state |= 0x04);
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
	motor1_write(0.0);
	if ((_control_state & 0x04) == 0x04)
		return (_control_state ^= 0x04);
	else
		return _control_state;
}

unsigned char control_get_state (void){
	return _control_state;
}

void control_av_wheel (void){
	const double epgain = 0.090, eigain = 0.0010, edgain = 0.000010, period = 0.020;

	double ep_r, ep_l;
	ep_r = (_control_t_av_wheel_r - odometry_get_av_rightwheel());
	ep_l = (_control_t_av_wheel_l - odometry_get_av_leftwheel());

	double ed_r, ed_l;
	ed_r = (_control_av_wheel_r_ep - ep_r)/period;
	ed_l = (_control_av_wheel_r_ep - ep_r)/period;

	_control_av_wheel_r_ep = ep_r;
	_control_av_wheel_l_ep = ep_l;

	_control_av_wheel_r_ei += ep_r*period;
	_control_av_wheel_l_ei += ep_l*period;

	motor3_write(epgain*_control_av_wheel_r_ep + eigain*_control_av_wheel_r_ei + edgain*ed_r);
	motor4_write(epgain*_control_av_wheel_l_ep + eigain*_control_av_wheel_l_ei + edgain*ed_l);
}

void control_follow_cubicCurve (void){
	static double u = 0.0;
	static int vel_grad = 0;

	const double v = (_control_cubicCurve_vel[vel_grad]+_control_cubicCurve_vel[vel_grad+1]*(4.0*u-vel_grad));
	const double fbgain_phi = 0.10, fbgain_y = 1.0/v;
	vec2 cp, t_cp;
	cp = num2vec2(odometry_get_x(), odometry_get_y());

	u = cubicCurve_get_nearVar(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, cp, u);

	if (4.0*u-vel_grad > 1.0)
		vel_grad++;

	t_cp = cubicCurve_get_point(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, u);

	volatile double d_phi = odometry_get_angle() - cubicCurve_get_vecAngle(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, u);
	volatile double d_y = (t_cp.v1-cp.v1)*sin(odometry_get_angle()) + (t_cp.v2-cp.v2)*cos(odometry_get_angle());

    if (d_phi < -PI)
    	d_phi += 2*PI;
    else if (d_phi > PI)
    	d_phi -= 2*PI;

    volatile double ff, curve_radius = cubicCurve_get_curvRad(_control_t_cubicCurve_coes_x, _control_t_cubicCurve_coes_y, u);
    if (isnan(curve_radius))
    	ff = 0.0;
    else
    	ff = v/curve_radius*WHEEL_DISTANCE/2.0;

    if (u < 1.0)
    	control_av_wheel_start ((v+ff-fbgain_phi*d_phi-fbgain_y*d_y)/WHEEL_RADIUS, (v-ff+fbgain_phi*d_phi+fbgain_y*d_y)/WHEEL_RADIUS);
    else{
    	control_av_wheel_end();
    	control_follow_cubicCurve_end();
    	u = 0.0;
    	vel_grad = 0;
    }
}
/*
void control_av_throwingArm (void){
	const double epgain = 1.0, eigain = 0.10, edgain = 0.010, period = 0.020;

	double ep = _control_t_av_throwingArm-odometry_get_av_throwingArm();

	double ed = (_control_av_throwingArm_ep-ep)/period;
	_control_av_throwingArm_ep = ep;
	_control_av_throwingArm_ei += ep*period;

	motor2_write(epgain*_control_av_throwingArm_ep + eigain*_control_av_throwingArm_ei + edgain*ed);
}
*/
