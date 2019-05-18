/*
 * body.c
 *
 *  Created on: 2019/05/14
 *      Author: Sano
 */

#include "body.h"
#include "xprintf.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

volatile double body_pos[3] = {}, body_vel[3] = {}, body_acc[3] = {};

void body_init (void){
	leg_initPWM_KEY_HPTR(&leg[0], &htim1, &htim2, &htim3);
	leg_initPWM_KEY_HPTR(&leg[1], &htim1, &htim3, &htim4);
	leg_initPWM_KEY_HPTR(&leg[2], &htim4, &htim4, &htim3);
	leg_initPWM_KEY_HPTR(&leg[3], &htim2, &htim2, &htim3);

	leg_initPWM_KEY_CHANNEL(&leg[0], TIM_CHANNEL_4, TIM_CHANNEL_4, TIM_CHANNEL_4);
	leg_initPWM_KEY_CHANNEL(&leg[1], TIM_CHANNEL_2, TIM_CHANNEL_2, TIM_CHANNEL_1);
	leg_initPWM_KEY_CHANNEL(&leg[2], TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_1);
	leg_initPWM_KEY_CHANNEL(&leg[3], TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);

	leg_initLENGTH(&leg[0], L1, L2, L3);
	leg_initLENGTH(&leg[1], L1, L2, L3);
	leg_initLENGTH(&leg[2], L1, L2, L3);
	leg_initLENGTH(&leg[3], L1, L2, L3);

	leg_initPULSEWIDTH_0DEG(&leg[0], 1510, 1480, 1480);
	leg_initPULSEWIDTH_0DEG(&leg[1], 1520, 1520, 1480);
	leg_initPULSEWIDTH_0DEG(&leg[2], 1470, 1450, 1450);
	leg_initPULSEWIDTH_0DEG(&leg[3], 1460, 1450, 1520);

	leg_initPULSEWIDTH_90DEG(&leg[0], 2150,  850, 2110);
	leg_initPULSEWIDTH_90DEG(&leg[1],  860,  900, 2140);
	leg_initPULSEWIDTH_90DEG(&leg[2], 2110, 2130,  780);
	leg_initPULSEWIDTH_90DEG(&leg[3],  795, 2110,  840);

	leg_initPOSTURE_DIR(&leg[0], 1, 1);
	leg_initPOSTURE_DIR(&leg[1], 1, -1);
	leg_initPOSTURE_DIR(&leg[2], -1, -1);
	leg_initPOSTURE_DIR(&leg[3], -1, 1);

	gr[0].x = LD/2.0;
	gr[0].y = LW/2.0;
	gr[0].z = 0.0;

	gr[1].x = -LD/2.0;
	gr[1].y = LW/2.0;
	gr[1].z = 0.0;

	gr[2].x = -LD/2.0;
	gr[2].y = -LW/2.0;
	gr[2].z = 0.0;

	gr[3].x = LD/2.0;
	gr[3].y = -LW/2.0;
	gr[3].z = 0.0;
}

double body_getPos_y (void){
	return body_pos[1];
}

double body_getPos_x (void){
	return body_pos[0];
}

double body_getVel_y (void){
	return body_vel[1];
}

double body_getVel_x (void){
	return body_vel[0];
}

Vec3 body_getLegPoint (int n){
	Vec3 r = v3_add(gr[n-1], leg[n-1].pos);

	return r;
}

Vec3 body_getLegPos (int n){
	return leg[n-1].pos;
}

double body_getLegAng (int n, int m){
    return leg[n-1].angle[m];
}

Vec3 body_getGP (void){
	// assume leg[n].angle[0] ~ 0.0
	volatile Vec3 ans;

	ans.x = L2G*(sin(leg[0].angle[1]) + sin(leg[1].angle[1]) + sin(leg[2].angle[1]) + sin(leg[3].angle[1]))*M_L2/(M_BODY+M_L2)/4.0;
    ans.x += (ans.x*L2/L2G*4.0*(M_BODY+M_L2)/M_L  + L3G*((sin(leg[0].angle[1]+leg[0].angle[2]) + sin(leg[1].angle[1]+leg[1].angle[2]) + sin(leg[2].angle[1]+leg[2].angle[2]) + sin(leg[3].angle[1]+leg[3].angle[2]))))*M_L3/(M_BODY+M_L3)/4.0;

	ans.y = 0.0;

	ans.z = -L2G*(cos(leg[0].angle[1]) + cos(leg[1].angle[1]) + cos(leg[2].angle[1]) + cos(leg[3].angle[1]))*M_L2/(M_BODY+M_L2)/4.0;
    ans.z += (ans.z*L2/L2G*4.0*(M_BODY+M_L2)/M_L  - L3G*((cos(leg[0].angle[1]+leg[0].angle[2]) + cos(leg[1].angle[1]+leg[1].angle[2]) + cos(leg[2].angle[1]+leg[2].angle[2]) + cos(leg[3].angle[1]+leg[3].angle[2]))))*M_L3/(M_BODY+M_L3)/4.0;

    return ans;
}

double body_getRotateRadiusOfBody (int n1, int n2){
	Vec3 s, l1g, sng;

	s = v3_sub(v3_add(gr[n2-1], leg[n2-1].pos), v3_add(gr[n1-1], leg[n1-1].pos));
	l1g = v3_mul_sclr(-1.0, v3_add(gr[n1-1], leg[n1-1].pos));
	sng = v3_sub(l1g, v3_mul_sclr(v3_dot(l1g, s)/v3_length(s), v3_normalize(s)));

	return v3_length(sng);
}

Vec3 body_getRotateRadiusVecOfBody (int n1, int n2){
	Vec3 s, l1g, sng;

	s = v3_sub(v3_add(gr[n2-1], leg[n2-1].pos), v3_add(gr[n1-1], leg[n1-1].pos));
	l1g = v3_mul_sclr(-1.0, v3_add(gr[n1-1], leg[n1-1].pos));
	sng = v3_sub(l1g, v3_mul_sclr(v3_dot(l1g, s)/v3_length(s), v3_normalize(s)));

	return sng;
}

double body_getMomentByG (int n1, int n2){
	volatile Vec3 s, l1g, sng, mnt;

	s = v3_sub(v3_add(gr[n2-1], leg[n2-1].pos), v3_add(gr[n1-1], leg[n1-1].pos));
	l1g = v3_sub(body_getGP(), v3_add(gr[n1-1], leg[n1-1].pos));
	sng = v3_sub(l1g, v3_mul_sclr(v3_dot(l1g, s)/v3_length(s), v3_normalize(s)));

	mnt = v3_closs(sng, v3_num2vec(0, 0, -GRAVITY));
	if (s.x*mnt.x < 0.0)
		return -v3_length(mnt);
	else
		return v3_length(mnt);
}

void body_addForce (double fx, double fy, double fz){
	body_acc[0] = fx/M_BODY;
	body_acc[1] = fy/M_BODY;
	body_acc[2] = fz/M_BODY;

	leg[0].acc.x = -fx/M_BODY;
	leg[1].acc.x = -fx/M_BODY;
	leg[2].acc.x = -fx/M_BODY;
	leg[3].acc.x = -fx/M_BODY;

	leg[0].acc.y = -fy/M_BODY;
	leg[1].acc.y = -fy/M_BODY;
	leg[2].acc.y = -fy/M_BODY;
	leg[3].acc.y = -fy/M_BODY;

	leg[0].acc.z = -fz/M_BODY;
	leg[1].acc.z = -fz/M_BODY;
	leg[2].acc.z = -fz/M_BODY;
	leg[3].acc.z = -fz/M_BODY;
}

void body_setVel (double vx, double vy, double vz){
	body_vel[0] = vx;
	body_vel[1] = vy;
	body_vel[2] = vz;

	leg[0].vel.x = -vx;
	leg[1].vel.x = -vx;
	leg[2].vel.x = -vx;
	leg[3].vel.x = -vx;

	leg[0].vel.y = -vy;
	leg[1].vel.y = -vy;
	leg[2].vel.y = -vy;
	leg[3].vel.y = -vy;

	leg[0].vel.z = -vz;
	leg[1].vel.z = -vz;
	leg[2].vel.z = -vz;
	leg[3].vel.z = -vz;
}

void body_setLegPos (int n, double x, double y, double z){
	leg_setPos(&leg[n-1], x, y, z);
}

void body_setLegVel (int n, double x, double y, double z){
	leg_setVel(&leg[n-1], x, y, z);
}

void body_setLegAcc (int n, double x, double y, double z){
	leg_setAcc(&leg[n-1], x, y, z);
}

void body_move (void){
	const double period = 0.010;

	body_pos[0] += body_vel[0]*period + body_acc[0]*period*period/2.0;
	body_pos[1] += body_vel[1]*period + body_acc[1]*period*period/2.0;
	body_pos[2] += body_vel[2]*period + body_acc[2]*period*period/2.0;

	body_vel[0] += body_acc[0]*period;
	body_vel[1] += body_acc[1]*period;
	body_vel[2] += body_acc[2]*period;

	leg_movement(&leg[0]);
	leg_movement(&leg[1]);
	leg_movement(&leg[2]);
	leg_movement(&leg[3]);

	leg_move(&leg[0]);
	leg_move(&leg[1]);
	leg_move(&leg[2]);
	leg_move(&leg[3]);
}
