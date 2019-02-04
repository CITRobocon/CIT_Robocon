/*
 * math_operations.c
 *
 *  Created on: 2019/01/20
 *      Author: Sano
 */

#include "math_operations.h"

#include "math.h"

vec2 arr2vec2 (double *p){
	vec2 vec;
	vec.v1 = *p;
	vec.v2 = *(p+1);

	return vec;
}

vec2 num2vec2 (double v1, double v2){
	vec2 vec;
	vec.v1 = v1;
	vec.v2 = v2;

	return vec;
}

vec4 arr2vec4 (double *p){
	vec4 vec;
	vec.v1 = *p;
	vec.v2 = *(p+1);
	vec.v3 = *(p+2);
	vec.v4 = *(p+3);

	return vec;
}

vec4 num2vec4 (double v1, double v2, double v3, double v4){
	vec4 vec;
	vec.v1 = v1;
	vec.v2 = v2;
	vec.v3 = v3;
	vec.v4 = v4;

	return vec;
}

vec4 cubicEq_LaGrange (double p1, double p2, double p3, double p4){
	vec4 coes;
	coes.v1 = (-9*p1 + 27*p2 - 27*p3 + 9*p4)/2.0;
	coes.v2 = (18*p1 - 45*p2 + 36*p3 - 9*p4)/2.0;
	coes.v3 = (-11*p1 + 18*p2 - 9*p3 + 2*p4)/2.0;
	coes.v4 = p1;
	return coes;
}

vec4 cubicCurve_bezier (double p1, double p2, double p3, double p4){
	vec4 coes;
	coes.v1 = -p1 + 3.0*p2 - 3.0*p3 + p4;
	coes.v2 = 3.0*p1 - 6.0*p2 + 3.0*p3;
	coes.v3 = -3.0*p1 + 3.0*p2;
	coes.v4 = p1;

	return coes;
}

vec2 cubicCurve_get_point (vec4 coes_x, vec4 coes_y, double u){
	vec2 co;
	co.v1 = coes_x.v1*u*u*u + coes_x.v2*u*u + coes_x.v3*u + coes_x.v4;
	co.v2 = coes_y.v1*u*u*u + coes_y.v2*u*u + coes_y.v3*u + coes_y.v4;

	return co;
}

double cubicCurve_get_vecAngle (vec4 coes_x, vec4 coes_y, double u){
	return atan2(3.0*coes_y.v1*u*u + 2.0*coes_y.v2*u + coes_y.v3,
			     3.0*coes_x.v1*u*u + 2.0*coes_x.v2*u + coes_x.v3);
}

double cubicCurve_get_curvRad (vec4 coes_x, vec4 coes_y, double u){
	double den = (3.0*coes_x.v1*u*u + 2.0*coes_x.v2*u + coes_x.v3)*(6.0*coes_y.v1*u + 2.0*coes_y.v2)
			     - (3.0*coes_y.v1*u*u + 2.0*coes_y.v2*u + coes_y.v3)*(6.0*coes_x.v1*u + 2.0*coes_x.v2);
	double temp = hypot(3.0*coes_y.v1*u*u + 2.0*coes_y.v2*u + coes_y.v3,
			 	 	    3.0*coes_x.v1*u*u + 2.0*coes_x.v2*u + coes_x.v3);

	if (den == 0.0)
		return NAN;
	else
		return temp*temp*temp/den;
}

double cubicCurve_get_nearVar (vec4 coes_x, vec4 coes_y, vec2 cp, double u){
	double kc[6] = {};
	double c_primeprime;

	kc[0] = 3.0*(coes_x.v1*coes_x.v1 + coes_y.v1*coes_y.v1);
	kc[1] = 5.0*(coes_x.v1*coes_x.v2 + coes_y.v1*coes_y.v2);
	kc[2] = (4.0*coes_x.v1*coes_x.v3 + 2.0*coes_x.v2*coes_x.v2
			     + 4.0*coes_y.v1*coes_y.v3 + 2.0*coes_y.v2*coes_y.v2);
	kc[3] = 3.0*(coes_x.v1*(coes_x.v4-cp.v1) + coes_x.v2*coes_x.v3
			     + coes_y.v1*(coes_y.v4-cp.v2) + coes_y.v2*coes_y.v3);
	kc[4] = 2.0*coes_x.v2*(coes_x.v4-cp.v1) + coes_x.v3*coes_x.v3
			 + 2.0*coes_y.v2*(coes_y.v4-cp.v2) + coes_y.v3*coes_y.v3;
	kc[5] = coes_x.v3*(coes_x.v4-cp.v1) + coes_y.v3*(coes_y.v4-cp.v2);

	for (int i = 0; i < 5; i++){
		c_primeprime = 5.0*kc[0]*u*u*u*u + 4.0*kc[1]*u*u*u + 3.0*kc[2]*u*u + 2.0*kc[3]*u + kc[4];
		if (c_primeprime != 0.0)
			u -= (kc[0]*u*u*u*u*u + kc[1]*u*u*u*u + kc[2]*u*u*u + kc[3]*u*u + kc[4]*u + kc[5])/c_primeprime;
	}

	return u;
}

double saturation (double x, double x_min, double x_max){
	if (x < x_min)
		return x_min;
	else if (x > x_max)
		return x_max;
	else
		return x;
}

double sinc (double x){
	if (x == 0.0)
		return 1.0;
	else
		return sin(x)/x;

}

