/*
 * walk.h
 *
 *  Created on: 2019/05/07
 *      Author: Sano
 */

#ifndef WALK_H_
#define WALK_H_

#include "math_operations.h"
#include "leg_operations.h"
#include "sensors.h"
#include "body.h"

#define WALK_CRAWL	0
#define WALK_TROT	1
#define WALK_STABLE 2

#define WALK_GAIT WALK_TROT

int walk_isGround (int);

int isRightside (int);

int walk_getMaxStage (void);

Vec3 walk_pathPoint_freeLeg (int, double);

Vec2 walk_getRotationAxis (void);

int walk_transition (void);

void walk_start (void);

void walk_tick (void);

void walk_control_balance_start(void);

int walk_isUnderControl_balance(void);

void walk_control_balance (void);

#endif /* WALK_H_ */
