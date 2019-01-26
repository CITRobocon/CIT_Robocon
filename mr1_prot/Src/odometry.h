/*
 * odometry.h
 *
 *  Created on: 2019/01/20
 *      Author: Sano
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

double odometry_get_x(void);
double odometry_get_y(void);
double odometry_get_angle(void);
double odometry_get_av_rightwheel(void);
double odometry_get_av_leftwheel(void);
double odometry_get_av_throwingArm(void);

void odometry_set_position(double x, double y, double angle);

void odometry_update(void);

#endif /* ODOMETRY_H_ */
