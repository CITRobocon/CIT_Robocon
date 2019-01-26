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
#define MOTOR_THROWING_ARM_MAX_T (1.34*G*0.01)
#define MOTOR_THROWING_ARM_MAX_S (21000*2.0*PI/60)
#define MOTOR_THROWING_ARM_TS ((1.34*G*0.01)/(21000*2.0*PI/60))

#define MOTOR_WHEEL_MAX_T (1.97/71*19)
#define MOTOR_WHEEL_MAX_S (128*71/19*2*PI/60)
#define MOTOR_WHEEL_TS (3.0*(1.97/71*19)/(128*71/19*2*PI/60))

// mass
#define M_SHAGAI 0.660
#define M_THROWING_ARM 1.0

// inertia
#define I_WHEEL 0.000129
#define I_THROWING_ARM 1.0

// design
#define THROWING_ARM_ENCODER_RATIO 1.0
#define THROWING_ARM_DISTANCE_GP 1.0

#endif /* BODY_H_ */
