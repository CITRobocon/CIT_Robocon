/*
 * control_operations.h
 *
 *  Created on: 2019/01/21
 *      Author: Sano
 */

#ifndef CONTROL_OPERATIONS_H_
#define CONTROL_OPERATIONS_H_

#include "math_operations.h"


unsigned char control_av_wheel_start (double, double, double, int);
  //arguments : angular velocity of right motor
  //                                left motor
unsigned char control_follow_cubicCurve_start (vec4, vec4, double, double, double, double, double, int);
unsigned char control_av_throwingArm_start (double, int);
  //arguments : angular velocity of throwing arm
  //            mode (0 : without shagai, 1 : with shagai)
unsigned char control_angle_throwingArm_start (double, int);

void control_follow_cubicCurve_invert (void);

unsigned char control_av_wheel_end (void);
unsigned char control_follow_cubicCurve_end (void);
unsigned char control_av_throwingArm_end (void);
unsigned char control_angle_throwingArm_end (void);

unsigned char control_get_state (void);
int isUnderControl (int);

void control_av_wheel (void);
void control_follow_cubicCurve (void);
void control_av_throwingArm (void);
void control_angle_throwingArm (void);

#endif /* CONTROL_OPERATIONS_H_ */
