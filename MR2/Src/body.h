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

#define L1	TO_METER(32)
#define L2	TO_METER(150)
#define L3	TO_METER(170)

#define L2G	TO_METER(75)
#define L3G	TO_METER(88.3)

#define M_BODY		TO_KILOGRAM(1493)
#define M_L1		TO_KILOGRAM(40)
#define M_L2		TO_KILOGRAM(175)
#define M_L3		TO_KILOGRAM(37)
#define M_L			(M_L1+M_L2+M_L3)
#define M_FULLBODY	(M_BODY+M_L)

// legs
Leg leg[4];

// vector
Vec3 gr[4];

void body_init (void);

double body_getPos_y (void);

double body_getPos_x (void);

double body_getVel_y (void);

double body_getVel_x (void);

Vec3 body_getLegPoint (int);

Vec3 body_getLegPos (int);

double body_getLegAng (int, int);

Vec3 body_getGP (void);

double body_getRotateRadiusOfBody (int, int);

Vec3 body_getRotateRadiusVecOfBody (int, int);

double body_getMomentByG (int, int);

void body_addForce (double, double, double);

void body_setVel (double, double, double);

void body_setLegPos (int, double, double, double);

void body_setLegVel (int, double, double, double);

void body_setLegAcc (int, double, double, double);

void body_move (void);

#endif /* BODY_H_ */
