/*
 * math_operations.c
 *
 *  Created on: 2019/05/14
 *      Author: Sano
 */

#include "math_operations.h"

//vec2 functions
Vec2 v2_arr2vec (double arr[2]){
	Vec2 vec;

		vec.x = arr[0];
		vec.y = arr[1];

		return vec;
}

Vec2 v2_num2vec (double x, double y){
	Vec2 vec;

	vec.x = x;
	vec.y = y;

	return vec;
}

Vec2 v2_add (Vec2 vec1, Vec2 vec2){
	Vec2 ans;

	ans.x = vec1.x + vec2.x;
	ans.y = vec1.y + vec2.y;

	return ans;
}

Vec2 v2_sub (Vec2 vec1, Vec2 vec2){
	Vec2 ans;

	ans.x = vec1.x - vec2.x;
	ans.y = vec1.y - vec2.y;

	return ans;
}

Vec2 v2_mul_sclr (double n, Vec2 vec){
	vec.x *= n;
	vec.y *= n;

	return vec;
}

Vec2 v2_rotate (Vec2 vec, double theta){
	Vec2 ans;

	ans.x = vec.x*cos(theta) - vec.y*sin(theta);
	ans.y = vec.x*sin(theta) + vec.y*cos(theta);

	return ans;
}

double v2_angle (Vec2 vec){
	return atan2(vec.y, vec.x);
}

double v2_length (Vec2 vec){
	return hypot(vec.x, vec.y);
}

//vec3 functions
Vec3 v3_arr2vec (double arr[3]){
	Vec3 vec;

	vec.x = arr[0];
	vec.y = arr[1];
	vec.z = arr[2];

	return vec;
}

Vec3 v3_num2vec (double x, double y, double z){
	Vec3 vec;

	vec.x = x;
	vec.y = y;
	vec.z = z;

	return vec;
}

Vec3 v3_add (Vec3 vec1, Vec3 vec2){
	Vec3 ans;

	ans.x = vec1.x + vec2.x;
	ans.y = vec1.y + vec2.y;
	ans.z = vec1.z + vec2.z;

	return ans;
}

Vec3 v3_sub (Vec3 vec1, Vec3 vec2){
	Vec3 ans;

	ans.x = vec1.x - vec2.x;
	ans.y = vec1.y - vec2.y;
	ans.z = vec1.z - vec2.z;

	return ans;
}

Vec3 v3_mul_sclr (double n, Vec3 vec){
	Vec3 ans;

	ans.x = n*vec.x;
	ans.y = n*vec.y;
	ans.z = n*vec.z;

	return ans;
}

Vec3 v3_closs (Vec3 vec1, Vec3 vec2){
	volatile Vec3 ans;

	ans.x = vec1.y*vec2.z - vec1.z*vec2.y;
	ans.y = -vec1.x*vec2.z + vec1.z*vec2.x;
	ans.z = vec1.x*vec2.y - vec1.y*vec2.x;

	return ans;
}

double v3_dot (Vec3 vec1, Vec3 vec2){
	return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z;
}

double v3_length (Vec3 vec){
	return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

Vec3 v3_normalize (Vec3 vec){
	double len = v3_length(vec);

	vec.x /= len;
	vec.y /= len;
	vec.z /= len;

	return vec;
}

double rodrigues_rp (Vec3 n, double roll, double pitch){
	double c = (cos(pitch)*cos(roll)-n.z*n.z)/(1.0-n.z*n.z);
	double s = (-sin(pitch)+n.x*n.z*(c-1.0))/n.y;

	return atan2(s,c);
}

//scalar functions
double satuation (double x, double x_min, double x_max){
	if (x < x_min)
		return x_min;
	if (x > x_max)
		return x_max;
	return x;
}

double sign (double x){
	if (x < 0.0)
		return -1.0;
	if (x > 0.0)
		return 1.0;
	return 0.0;
}
