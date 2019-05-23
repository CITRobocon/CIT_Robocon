/*
 * body.h
 *
 *  Created on: 2019/01/20
 *      Author: Sano
 */

#ifndef BODY_H_
#define BODY_H_

#define G 9.81

// electrical characteristic of motor
#define MOTOR_THROWING_ARM_MAX_T (11.1*G*0.010*84*0.60)
#define MOTOR_THROWING_ARM_MAX_S (21000*2.0*PI/60)
#define MOTOR_THROWING_ARM_TS ((1.34*G*0.01)/(21000*2.0*PI/60))

#define MOTOR_WHEEL_MAX_T (1.97/71*19)
#define MOTOR_WHEEL_MAX_S (128*71/19*2*PI/60)
#define MOTOR_WHEEL_TS (3.0*(1.97/71*19)/(128*71/19*2*PI/60))

// mass
#define M_SHAGAI 0.660
#define M_THROWING_ARM 1.419691

// inertia
#define I_WHEEL 0.000129
#define I_THROWING_ARM 0.281587727
#define I_THROWING_ARM_WITH_SHAGAI 0.758285147

// design
#define WHEEL_DISTANCE 0.424
#define WHEEL_RADIUS (0.085/2.0)

#define ENCODER_DISTANCE (0.619)
#define ENCODER_RADIUS (0.051/2.0)

#define THROWING_ARM_ENCODER_RATIO 1.0
#define THROWING_ARM_DISTANCE_GP 1.0

#define L_SG_THROWING_ARM 0.281587727
#define L_SG_THROWING_ARM_WITH_SHAGAI 0.758285147

#endif /* BODY_H_ */
