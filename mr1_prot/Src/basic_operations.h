/*
 * basic_operations.h
 *
 *  Created on: 2019/01/18
 *      Author: Sano
 */

#ifndef BASIC_OPERATIONS_H_
#define BASIC_OPERATIONS_H_

// write motors
void motor1_write(double);
void motor2_write(double);
void motor3_write(double);
void motor4_write(double);

// get encoders' value
short enc_right_getAndResetCount(void);

short enc_left_getAndResetCount(void);

void enc_arm_updateCount(void);
void enc_arm_setAngle_0rad(void);
double enc_arm_getAngle_rad(void);

// get switches' value
int sw1_getState(void);
int sw2_getState(void);
int sw2_getState(void);
int PR_getState(void);

// write leds
void led3c_write(int, int);
void led1_write(int);
void led2_write(int);

#endif /* BASIC_OPERATIONS_H_ */
