/*
 * math_operations.h
 *
 *  Created on: 2019/05/06
 *      Author: Sano
 */

#ifndef MATH_OPERATIONS_H_
#define MATH_OPERATIONS_H_

#include <math.h>

#define PI 3.14159265358979323846264338327950288419716939937510582

#define MATH_DEG_TO_RAD(x) ((x)*PI/180)
#define MATH_RAD_TO_DEG(x) ((x)*180/PI)

/*
#define X 0
#define Y 1
#define Z 2
*/

typedef struct{
	double x;
	double y;
}Vec2;

typedef struct{
	double x;
	double y;
	double z;
}Vec3;

//vec2 functions
Vec2 v2_arr2vec (double arr[2]);

Vec2 v2_num2vec (double x, double y);

Vec2 v2_add (Vec2 vec1, Vec2 vec2);

Vec2 v2_sub (Vec2 vec1, Vec2 vec2);

Vec2 v2_mul_sclr (double n, Vec2 vec);

Vec2 v2_rotate (Vec2 vec, double theta);

double v2_angle (Vec2 vec);

double v2_length (Vec2 vec);

//vec3 functions
Vec3 v3_arr2vec (double arr[3]);

Vec3 v3_num2vec (double x, double y, double z);

Vec3 v3_add (Vec3 vec1, Vec3 vec2);

Vec3 v3_sub (Vec3 vec1, Vec3 vec2);

Vec3 v3_mul_sclr (double n, Vec3 vec);

Vec3 v3_closs (Vec3 vec1, Vec3 vec2);

double v3_dot (Vec3 vec1, Vec3 vec2);

double v3_length (Vec3 vec);

Vec3 v3_normalize (Vec3 vec);

//scalar functions
double satuation (double x, double x_min, double x_max);

double sign (double x);

#endif /* MATH_OPERATIONS_H_ */
