/*
 * leg_operations.c
 *
 *  Created on: 2019/05/14
 *      Author: Sano
 */

#include "leg_operations.h"

// functions for initialize structures "Leg"
void leg_initPWM_KEY_HPTR (Leg *leg, TIM_HandleTypeDef *hptr1, TIM_HandleTypeDef *hptr2, TIM_HandleTypeDef *hptr3){
	PwmKey key[3] = {{hptr1, leg->pwm_key[JOINT1].channel}, {hptr2, leg->pwm_key[JOINT2].channel}, {hptr3, leg->pwm_key[JOINT3].channel}};
	for (int i = 0; i < 3; i++)
		leg->pwm_key[i] = key[i];
}

void leg_initPWM_KEY_CHANNEL (Leg *leg, uint32_t channel1, uint32_t channel2, uint32_t channel3){
	PwmKey key[3] = {{leg->pwm_key[0].hptr, channel1}, {leg->pwm_key[1].hptr, channel2}, {leg->pwm_key[2].hptr, channel3}};
	for (int i = 0; i < 3; i++)
		leg->pwm_key[i] = key[i];
}

void leg_initLENGTH (Leg *leg, double l1, double l2, double l3){
	leg->length[ARM1] = l1;
	leg->length[ARM2] = l2;
	leg->length[ARM3] = l3;
}

void leg_initPULSEWIDTH_0DEG (Leg *leg, double n1, double n2, double n3){
	leg->pulsewidth_0deg[JOINT1] = n1;
	leg->pulsewidth_0deg[JOINT2] = n2;
	leg->pulsewidth_0deg[JOINT3] = n3;
}

void leg_initPULSEWIDTH_90DEG (Leg *leg, double n1, double n2, double n3){
	leg->pulsewidth_90deg[JOINT1] = n1;
	leg->pulsewidth_90deg[JOINT2] = n2;
	leg->pulsewidth_90deg[JOINT3] = n3;
}

void leg_initPOSTURE_DIR (Leg *leg, int m, int n){
	leg->posture_dir[0] = m;
	leg->posture_dir[1] = n;
}


// functions for control legs
void leg_setJointAngle_rad (Leg *leg, const double ang1, const double ang2, const double ang3){
	leg->angle[JOINT1] = ang1;
	leg->angle[JOINT2] = ang2;
	leg->angle[JOINT3] = ang3;
	__HAL_TIM_SetCompare(leg->pwm_key[JOINT1].hptr,leg->pwm_key[JOINT1].channel,(leg->pulsewidth_90deg[JOINT1]-leg->pulsewidth_0deg[JOINT1])/PI*2.0*ang1+leg->pulsewidth_0deg[JOINT1]);
	__HAL_TIM_SetCompare(leg->pwm_key[JOINT2].hptr,leg->pwm_key[JOINT2].channel,(leg->pulsewidth_90deg[JOINT2]-leg->pulsewidth_0deg[JOINT2])/PI*2.0*ang2+leg->pulsewidth_0deg[JOINT2]);
	__HAL_TIM_SetCompare(leg->pwm_key[JOINT3].hptr,leg->pwm_key[JOINT3].channel,(leg->pulsewidth_90deg[JOINT3]-leg->pulsewidth_0deg[JOINT3])/PI*2.0*ang3+leg->pulsewidth_0deg[JOINT3]);
}

void leg_setPos (Leg *leg, const double px, const double py, const double pz){
	leg->pos.x = px;
	leg->pos.y = py;
	leg->pos.z = pz;
}

void leg_setVel (Leg *leg, const double vx, const double vy, const double vz){
	leg->vel.x = vx;
	leg->vel.y = vy;
	leg->vel.z = vz;
}

void leg_setAcc (Leg *leg, const double ax, const double ay, const double az){
	leg->acc.x = ax;
	leg->acc.y = ay;
	leg->acc.z = az;
}

void leg_movement (Leg *leg){
	const double period = 0.020;

	leg->pos = v3_add(leg->pos, v3_add(v3_mul_sclr(period, leg->vel), v3_mul_sclr(period*period/2.0, leg->acc)));

	leg->vel = v3_add(leg->vel, v3_mul_sclr(period, leg->acc));
}

void leg_move (Leg *leg){
	volatile double tempC, tempS, ang1, ang2, ang3;
	tempC = (-(leg->pos.x*leg->pos.x) - (leg->pos.y*leg->pos.y) - (leg->pos.z*leg->pos.z) + (leg->length[ARM1]*leg->length[ARM1]) + (leg->length[ARM2]*leg->length[ARM2]) + (leg->length[ARM3]*leg->length[ARM3]))/2.0/leg->length[ARM2]/leg->length[ARM3];
	ang3 = leg->posture_dir[1]*(PI-acos(tempC));

	tempS = leg->length[ARM3]*sin(ang3)/sqrt(leg->pos.x*leg->pos.x + leg->pos.y*leg->pos.y + leg->pos.z*leg->pos.z - leg->length[ARM1]*leg->length[ARM1]);
	ang2 = -(asin(tempS) - asin(leg->pos.x/sqrt(leg->pos.x*leg->pos.x + leg->pos.y*leg->pos.y + leg->pos.z*leg->pos.z - leg->length[ARM1]*leg->length[ARM1])));

	//ang1 = atan2(ry, -rz) - atan2(_leg_length[0], _leg_length[1]*cos(ang2)+_leg_length[2]*cos(ang2+ang3));
	ang1 = atan(leg->posture_dir[0]*leg->pos.y/-leg->pos.z) - atan(leg->length[ARM1]/(leg->length[ARM2]*cos(ang2)+leg->length[ARM3]*cos(ang2+ang3)));

	leg_setJointAngle_rad(leg, ang1, ang2, ang3);
}
