/*
 * math_operations.h
 *
 *  Created on: 2019/01/20
 *      Author: Sano
 */

#ifndef MATH_OPERATIONS_H_
#define MATH_OPERATIONS_H_

#define PI 3.1415926535897932384626433832795028841971693993

typedef struct{
	double v1;
	double v2;
}vec2;

typedef struct{
	double v1;
	double v2;
	double v3;
	double v4;
}vec4;

vec2 arr2vec2 (double*);
vec2 num2vec2 (double, double);

vec4 arr2vec4 (double*);
vec4 num2vec4 (double, double, double, double);


vec4 cubicCurve_bezier (double, double, double, double);

vec2   cubicCurve_get_point (vec4, vec4, double);
double cubicCurve_get_vecAngle (vec4, vec4, double);
double cubicCurve_get_curvRad (vec4, vec4, double);
double cubicCurve_get_nearVar (vec4, vec4, vec2, double);


double saturation (double, double, double);

#endif /* MATH_OPERATIONS_H_ */
