/*
 * basic_operation.c
 *
 *  Created on: 2019/01/19
 *      Author: Sano
 */

#include "basic_operations.h"

#include "stm32f4xx_hal.h"
#include "math_operations.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// write motors
void motor1_write (double ratio){
	ratio = saturation(ratio, -1.0, 1.0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (int)(250*ratio+750));
}

void motor2_write (double ratio){
	ratio = saturation(ratio, -1.0, 1.0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (int)(250*ratio+750));
}

void motor3_write (double ratio){
	ratio = saturation(ratio, -1.0, 1.0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (int)(250*ratio+750));
}

void motor4_write (double ratio){
	ratio = saturation(ratio, -1.0, 1.0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (int)(250*ratio+750));
}

// get encoders' value
short enc_right_getAndResetCount (void){
	volatile const uint16_t bits = TIM3->CNT;
	const short val = ((bits>>15) ? -(0xFFFF-bits)-1 : bits);

	TIM3->CNT -= val;

	return val;
}

short enc_left_getAndResetCount (void){
	volatile const uint16_t bits = TIM1->CNT;
	const short val = ((bits>>15) ? -(0xFFFF-bits)-1 : bits);

	TIM1->CNT -= val;

	return val;
}

volatile long _enc_arm_count = 0;
volatile double _enc_arm_av;

void enc_arm_updateCount (void){
	const double period = 0.010, ppr = 400*84;

	volatile const uint16_t bits = TIM4->CNT;
	const short val = ((bits>>15) ? -(0xFFFF-bits)-1 : bits);

	TIM4->CNT -= val;
	_enc_arm_count += val;
	_enc_arm_av = (double)(val)/ppr*2.0*PI/period;
}

void enc_arm_setAngle_rad (double angle){
	const double ppr = 400*84;
	TIM4->CNT = 0;
	_enc_arm_count = (long)(angle*ppr/2.0/PI);
}

double enc_arm_getAV (void){
	return _enc_arm_av;
}

double enc_arm_getAngle_rad (void){
	const double ppr = 400*84;
	return (_enc_arm_count/ppr*2.0*PI);
}

// get switches' value
int sw1_getState (void){
	return HAL_GPIO_ReadPin(GPIOC, SWITCH_Pin);
}

int sw2_getState (void){
	return HAL_GPIO_ReadPin(GPIOC, SWITCH2_Pin);
}

int sw3_getState (void){
	return HAL_GPIO_ReadPin(GPIOC, SWITCH3_Pin);
}

int remote_sw_getState (void){
	return HAL_GPIO_ReadPin(GPIOC, REMOTE_SWITCH_Pin);
}

// write led
void led3c_write (int in1, int in2){
    HAL_GPIO_WritePin(GPIOC, LED_THREE_1_Pin, in1&1);
	HAL_GPIO_WritePin(GPIOC, LED_THREE_2_Pin, in2&1);
}

void led1_write (int in){
	HAL_GPIO_WritePin(GPIOC, LED1_Pin, in&1);
}

void led2_write (int in){
    HAL_GPIO_WritePin(GPIOA, LED2_Pin, in&1);
}

// solenoid
void solenoid_toggle (void){
	HAL_GPIO_TogglePin(GPIOC, SOLENOID_OUT_Pin);
}

