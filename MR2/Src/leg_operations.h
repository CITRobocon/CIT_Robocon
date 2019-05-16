/*
 * leg_operations.h
 *
 *  Created on: 2019/05/05
 *      Author: Sano
 */

#ifndef LEG_OPERATIONS_H_
#define LEG_OPERATIONS_H_

#include "stm32f4xx_hal.h"

#include "math_operations.h"

#define JOINT1 0
#define JOINT2 1
#define JOINT3 2

#define ARM1 0
#define ARM2 1
#define ARM3 2

typedef struct{
	TIM_HandleTypeDef *hptr;
	uint32_t channel;
}PwmKey;

typedef struct{
	//mechanic variables
	volatile double length[3];			//ARM1,2,3
	volatile PwmKey pwm_key[3];			//JOINT1,2,3
	volatile double pulsewidth_0deg[3];	//JOINT1,2,3
	volatile double pulsewidth_90deg[3];	//JOINT1,2,3
	volatile int posture_dir[2];			//JOINT1,3

	//virtual variables
	volatile double angle[3];			//JOINT1,2,3
	volatile Vec3 pos;   				//X,Y,Z
	volatile Vec3 vel;					//X,Y,Z
	volatile Vec3 acc;					//X,Y,Z
}Leg;


// functions for initialize structures "Leg"
void leg_initPWM_KEY_HPTR (Leg *leg, TIM_HandleTypeDef *hptr1, TIM_HandleTypeDef *hptr2, TIM_HandleTypeDef *hptr3);

void leg_initPWM_KEY_CHANNEL (Leg *leg, uint32_t channel1, uint32_t channel2, uint32_t channel3);

void leg_initLENGTH (Leg *leg, double l1, double l2, double l3);

void leg_initPULSEWIDTH_0DEG (Leg *leg, double n1, double n2, double n3);

void leg_initPULSEWIDTH_90DEG (Leg *leg, double n1, double n2, double n3);

void leg_initPOSTURE_DIR (Leg *leg, int m, int n);

// functions for control legs
void leg_setJointAngle_rad (Leg *leg, const double ang1, const double ang2, const double ang3);

void leg_setPos (Leg *leg, const double px, const double py, const double pz);

void leg_setVel (Leg *leg, const double vx, const double vy, const double vz);

void leg_setAcc (Leg *leg, const double ax, const double ay, const double az);

void leg_movement (Leg *leg);

void leg_move (Leg *leg);

#endif /* LEG_OPERATIONS_H_ */
