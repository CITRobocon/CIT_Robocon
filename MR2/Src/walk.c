/*
 * walk.c
 *
 *  Created on: 2019/05/15
 *      Author: Sano
 */

#include "walk.h"
#include "xprintf.h"

volatile char leg_state = 0x0F; // 1 1 1 1 (all legs are grounding)
volatile int walking = 0;
volatile double internal_time = 0.0;

volatile int control_balance = 0;
/*
double walk_period = 0.40;		// sec
double walk_speed = 0.70;   	// m/s
double walk_hight = 0.25;		// m
double walk_legInward = -0.09;	// m
*/

double walk_period = 0.50;		// sec
double walk_speed = 0.0;   	// m/s
double walk_hight = 0.25;		// m
double walk_legInward = 0.0;	// m


int walk_isGround (int n){
	return ((leg_state>>(n-1))&0x01);
}

int isRightside (int n){
	if (n < 3)
		return 0;
	else
		return 1;
}

int walk_getMaxStage (void){
	if (WALK_GAIT ==WALK_CRAWL)
		return 4;
	else if (WALK_GAIT == WALK_TROT)
		return 2;
	else if (WALK_GAIT == WALK_STABLE)
		return 4;
	else
		return 0;
}

Vec3 walk_pathPoint_freeLeg (int n, double u){	// walk_pathPoint_freeLeg(leg_number, 0.0~1.0)
	const double hightrate = 0.5;
	const double pz[4] = {-walk_hight, -walk_hight*hightrate, -walk_hight*hightrate, -walk_hight};
	const Vec3 legPos = body_getLegPos(n);

	if (isRightside(n))
		return v3_num2vec(u*walk_speed*walk_period*(walk_getMaxStage()-1)/walk_getMaxStage() - walk_speed*walk_period*(walk_getMaxStage()-1)/walk_getMaxStage()/2.0,
				          (walk_legInward-legPos.y)*u+legPos.y,
			              (-pz[0]+3.0*pz[1]-3.0*pz[2]+pz[3])*u*u*u + (3.0*pz[0]-6.0*pz[1]+3.0*pz[2])*u*u + (-3.0*pz[0]+3.0*pz[1])*u + pz[0]);
	else
		return v3_num2vec(u*walk_speed*walk_period*(walk_getMaxStage()-1)/walk_getMaxStage() - walk_speed*walk_period*(walk_getMaxStage()-1)/walk_getMaxStage()/2.0,
				          (-walk_legInward-legPos.y)*u+legPos.y,
			              (-pz[0]+3.0*pz[1]-3.0*pz[2]+pz[3])*u*u*u + (3.0*pz[0]-6.0*pz[1]+3.0*pz[2])*u*u + (-3.0*pz[0]+3.0*pz[1])*u + pz[0]);

}

Vec2 walk_getRotationAxis (void){	// 2 leg-numbers and state return. (x < y, z : stable = 1, unstable = 0)
	if (WALK_GAIT == WALK_CRAWL){
		Vec3 legPoint[4], gp;
		volatile Vec2 legPoint_xy[3], gp_xy, line_vec[3];
		int n = 0, i, exc_num = 5;

		gp = body_getGP();
		gp_xy = v2_num2vec(gp.x, gp.y);

		for (i = 1; i < 5; i++){
			if (walk_isGround(i)){
				legPoint[n] = body_getLegPoint(i);
				n++;
			}else
				exc_num = i;
		}

		for (i = 0; i < 3; i++)
			legPoint_xy[i] = v2_num2vec(legPoint[i].x, legPoint[i].y);

		for (i = 0; i < 2; i++)
			line_vec[i] = v2_sub(legPoint_xy[i+1], legPoint_xy[i]);
		line_vec[i] = v2_sub(legPoint_xy[0], legPoint_xy[i]);

		for (i = 0; i < 2; i++){
			if (v2_rotate(v2_sub(gp_xy, legPoint_xy[i]), -v2_angle(line_vec[i])).y < 0.0){
				if (i+1 >= exc_num)
					return v2_num2vec(i+2, i+3);
				else if (i+2 >= exc_num)
					return v2_num2vec(i+1, i+3);
				return v2_num2vec(i+1, i+2);
			}
		}
		if (v2_rotate(v2_sub(gp_xy, legPoint_xy[i]), -v2_angle(line_vec[i])).y < 0.0){
			if (i+1 >= exc_num)
				return v2_num2vec(1, i+1);
			else if (1 >= exc_num)
				return v2_num2vec(2, i);
			return v2_num2vec(1, i);
		}

		return v2_num2vec(0, 0);

    }else if (WALK_GAIT == WALK_TROT){
		if (walk_isGround(1))
			return v2_num2vec(1, 3);
		else
			return v2_num2vec(2, 4);

    }else{
		return v2_num2vec(0, 0);
	}
}

int walk_transition (void){		// new stage number return.
	int gait = WALK_GAIT;
	switch (gait){
	case WALK_CRAWL:
		if(!walk_isGround(1)){
			leg_state = 0x0B;	// 1 1 1 0 -> 1 0 1 1	stage1 -> 2
			return 2;
		}else if (!walk_isGround(2)){
			leg_state = 0x0E;	// 1 1 0 1 -> 1 1 1 0   stage0 -> 1
			return 1;
		}else if (!walk_isGround(3)){
			leg_state = 0x07;	// 1 0 1 1 -> 0 1 1 1   stage2 -> 3
			return 3;
		}else{
			leg_state = 0x0D;	// 0 1 1 1 -> 1 1 0 1   stage3 -> 0
			return 0;
		}

	case WALK_TROT:
	    leg_state = (~leg_state)&0x0F;	// 1 0 1 0 <-> 0 1 0 1  stage0 <-> 1
	    return (leg_state&1);

	case WALK_STABLE:
		if(!walk_isGround(1)){
			leg_state = 0x0B;	// 1 1 1 0 -> 1 0 1 1	stage1 -> 2
			return 2;
		}else if (!walk_isGround(2)){
			leg_state = 0x0E;	// 1 1 0 1 -> 1 1 1 0   stage0 -> 1
			return 1;
		}else if (!walk_isGround(3)){
			leg_state = 0x07;	// 1 0 1 1 -> 0 1 1 1   stage2 -> 3
			return 3;
		}else{
			leg_state = 0x0D;	// 0 1 1 1 -> 1 1 0 1   stage3 -> 0
			return 0;
		}

	default:
		return 0;
	}
}

void walk_start (void){
	/*
	body_setLegPos(1, 0.0, -walk_legInward, -walk_hight);
	body_setLegPos(2, 0.0, -walk_legInward, -walk_hight);
	body_setLegPos(3, 0.0, walk_legInward, -walk_hight);
	body_setLegPos(4, 0.0, walk_legInward, -walk_hight);
	*/
	body_move();

	const int gait = WALK_GAIT;
	switch (gait){
	case WALK_CRAWL:
		walking = 1;
		internal_time = 0.0;
		leg_state = 0x0D;
		return;

	case WALK_TROT:
		walking = 1;
		internal_time = 0.0;
		leg_state = 0x0A;
		return;

	case WALK_STABLE:
		walking = 1;
		internal_time = 0.0;
		leg_state = 0x0D;
		return;

	default:
		leg_state = 0x0F;
		return;
	}
}

void walk_tick (void){
	if (!walking) return;

	const double period = 0.010;
	static int walk_stage = 0;

	if (WALK_GAIT == WALK_CRAWL){
		const double switching_time = walk_period/4.0;

		if ((internal_time - walk_stage*switching_time) >= switching_time)
			walk_stage = walk_transition();

		// cancel gravity
		volatile Vec2 legNumber = walk_getRotationAxis();
		double mnt, lg = 1.0;

		if (legNumber.x == 0.0 && legNumber.y == 0.0)
			mnt = 0.0;
		else if (legNumber.x == 1){
			mnt = body_getMomentByG((int)legNumber.x, (int)legNumber.y);
			lg = body_getRotateRadiusOfBody((int)legNumber.x, (int)legNumber.y);
		}else{
			mnt = body_getMomentByG((int)legNumber.y, (int)legNumber.x);
			lg = body_getRotateRadiusOfBody((int)legNumber.y, (int)legNumber.x);
		}

		// down leg
		/*
		body_setVel(walk_speed, 0.0, 0.0);
		if (mnt != 0.0){
			Vec3 rotateVec = v3_sub(body_getLegPoint(legNumber.x), body_getLegPoint(legNumber.y));
			Vec2 rotateVec2 = v2_num2vec(rotateVec.y, rotateVec.x);
			body_addForce(0.0, -1.0*mnt/lg/fabs(cos(v2_angle(rotateVec2))), 0.0);
		}
		*/

		// up leg
		Vec3 freePoint;

		volatile int i;
		for (i = 1; i < 5; i++){
			if (!walk_isGround(i))
				break;
		}
		freePoint = walk_pathPoint_freeLeg(i, internal_time/switching_time - walk_stage);
		/*
		body_setLegPos(i, freePoint.x, freePoint.y, freePoint.z);
		body_setLegVel(i, 0.0, 0.0, 0.0);
		body_setLegAcc(i, 0.0, 0.0, 0.0);
		*/

		internal_time += period;
		if (internal_time >= walk_period){
			walk_stage = 0;
			walk_start();
		}

	}else if (WALK_GAIT == WALK_TROT){
		const double switching_time = walk_period/2.0;

		if ((internal_time - walk_stage*switching_time) >= switching_time)
			walk_stage = walk_transition();

		// cancel gravity
		/*
		volatile double mnt, lg = 1.0;
		volatile Vec3 rotateVec;

		if (walk_stage == 1){
			mnt = body_getMomentByG(1,3);
			lg = body_getRotateRadiusOfBody(1,3);
			rotateVec = v3_sub(body_getLegPoint(1), body_getLegPoint(3));
		}else{
			mnt = body_getMomentByG(4,2);
			lg = body_getRotateRadiusOfBody(4,2);
			rotateVec = v3_sub(body_getLegPoint(4), body_getLegPoint(2));
		}
		*/

		// down leg
		//body_setVel(walk_speed, 0.0, 0.0);
		//volatile Vec2 rotateVec2 = v2_num2vec(rotateVec.y, rotateVec.x);
		//body_addForce(0.0, mnt/lg/fabs(cos(v2_angle(rotateVec2))), 0.0);

		// up leg
		Vec3 freePoint;

		if (walk_isGround(1)){
			freePoint = walk_pathPoint_freeLeg(2, internal_time/switching_time - walk_stage);
			body_setLegPos(2, freePoint.x, freePoint.y, freePoint.z);
			body_setLegVel(2, 0.0, 0.0, 0.0);
			body_setLegAcc(2, 0.0, 0.0, 0.0);

			freePoint = walk_pathPoint_freeLeg(4, internal_time/switching_time - walk_stage);
			body_setLegPos(4, freePoint.x, freePoint.y, freePoint.z);
			body_setLegVel(4, 0.0, 0.0, 0.0);
			body_setLegAcc(4, 0.0, 0.0, 0.0);
		}/*else{
			freePoint = walk_pathPoint_freeLeg(1, internal_time/switching_time - walk_stage);
			body_setLegPos(1, freePoint.x, freePoint.y, freePoint.z);
			body_setLegVel(1, 0.0, 0.0, 0.0);
			body_setLegAcc(1, 0.0, 0.0, 0.0);

			freePoint = walk_pathPoint_freeLeg(3, internal_time/switching_time - walk_stage);
			body_setLegPos(3, freePoint.x, freePoint.y, freePoint.z);
			body_setLegVel(3, 0.0, 0.0, 0.0);
			body_setLegAcc(3, 0.0, 0.0, 0.0);
		}*/

		internal_time += period;
		if (internal_time >= walk_period){
			walk_stage = 0;
			walk_start();
		}

	}else if (WALK_GAIT == WALK_STABLE){						// stable walk
		const double gain = 6.0;
		int i;
		Vec3 gp = body_getGP();
		volatile Vec2 tgp2 = v2_num2vec(0.0, 0.0), gp2 = v2_num2vec(gp.x, gp.y), ev;
		for (i = 1; i < 5; i++){
			if (walk_isGround(i)){
				tgp2.x += body_getLegPoint(i).x/3.0;
				tgp2.y += body_getLegPoint(i).y/3.0;
			}
		}

		ev = v2_sub(gp2, tgp2);
		ev.x *= 0.5;
		ev.y *= 0.3;

		// down leg
		body_setVel(-gain*ev.x, -gain*ev.y, 0.0);

		// up leg
		Vec3 freePoint;

		if (v2_length(ev) < 0.050){
			for (i = 1; i < 5; i++){
				if (!walk_isGround(i))
					break;
			}
			freePoint = walk_pathPoint_freeLeg(i, internal_time/walk_period);
			body_setLegPos(i, freePoint.x, freePoint.y, freePoint.z);
			body_setLegVel(i, 0.0, 0.0, 0.0);
			body_setLegAcc(i, 0.0, 0.0, 0.0);
			internal_time += period;
			if (internal_time >= walk_period){
				walk_stage = walk_transition();
				internal_time = 0.0;
			}
		}
	}
}

void walk_control_balance_start(void){
	control_balance = 1;
}

int walk_isUnderControl_balance(void){
	return control_balance;
}

void walk_control_balance (void){
	const double period = 0.010;

	const double fbGain[] = {113.6334, 15.8527, 40.6408, 19.6614};

	static double theta = 0.0;

	volatile double pre_theta = theta, theta_dot, x, x_dot, input;

	//Vec3 rotateRadVec = body_getRotateRadiusVecOfBody(1,3);
	Vec3 rotateVec = v3_sub(body_getLegPoint(1), body_getLegPoint(3));
	Vec2 rotateVec2 = v2_num2vec(rotateVec.x, rotateVec.y);

	theta = -rodrigues_rp(v3_normalize(rotateVec), gyro_getRoll(), gyro_getPitch());
	theta_dot = (theta-pre_theta)/period;
	if (body_getPos_y() < 0)
		x = hypot(body_getPos_x(), body_getPos_y());
	else
		x = -hypot(body_getPos_x(), body_getPos_y());

	if (body_getVel_y() < 0)
		x_dot = hypot(body_getVel_x(), body_getVel_y());
	else
		x_dot = -hypot(body_getVel_x(), body_getVel_y());

	input = -(fbGain[0]*x + fbGain[1]*x_dot + fbGain[2]*theta + fbGain[3]*theta_dot);

	//xprintf ("x, x dot, theta, theta dot = %d,\t%d,\t%d,\t%d\n", (int)(x*1000), (int)(x_dot*1000), (int)MATH_RAD_TO_DEG(theta), (int)MATH_RAD_TO_DEG(theta_dot));

	double tilt = (body_getLegAng(1, 0)+body_getLegAng(3, 0));

	xprintf ("%d\n", (int)MATH_RAD_TO_DEG(tilt));

	body_setVel(0.0, 0.0, 0.0);
	body_addForce(input*fabs(sin(v2_angle(rotateVec2)))*cos(tilt), -input*fabs(cos(v2_angle(rotateVec2)))*cos(tilt), input*sin(tilt));
	body_setLegAcc(2, 0.0, 0.0, 0.0);
	body_setLegAcc(4, 0.0, 0.0, 0.0);
}
