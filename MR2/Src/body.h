/*
 * full_body.h
 *
 *  Created on: 2019/05/06
 *      Author: Sano
 */

#ifndef BODY_H_
#define BODY_H_

#include "stm32f4xx_hal.h"

#include "leg_operations.h"
#include "math_operations.h"

#define GRAVITY 9.81

#define TO_METER(mm)	(mm*0.001)
#define TO_KILOGRAM(g)	(g*0.001)

#define LW	TO_METER(154)
#define LD	TO_METER(419.1)

#define L1	TO_METER(28)
#define L2	TO_METER(150)
#define L3	TO_METER(170)

#define L2G	TO_METER(75)
#define L3G	TO_METER(88.3)

#define M_BODY	TO_KILOGRAM(1493)
#define M_L1	TO_KILOGRAM(40)
#define M_L2	TO_KILOGRAM(175)
#define M_L3	TO_KILOGRAM(37)
#define M_L		(M_L1+M_L2+M_L3)

// legs
Leg leg[4];

// vector
Vec3 gr[4];

void body_init (void);

Vec3 body_getLegPoint (int n);

Vec3 body_getLegPos (int n);

Vec3 body_getGP (void);

double body_getRotateRadiusOfBody (int n1, int n2);

double body_getMomentByG (int n1, int n2);

void body_addForce (double fx, double fy, double fz);

void body_setVel (double vx, double vy, double vz);

void body_setLegPos (int n, double x, double y, double z);

void body_setLegVel (int n, double x, double y, double z);

void body_setLegAcc (int n, double x, double y, double z);

void body_move (void);

#endif /* BODY_H_ */
