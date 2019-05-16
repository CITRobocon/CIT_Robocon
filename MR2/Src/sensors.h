/*
 * sensors.h
 *
 *  Created on: 2019/05/14
 *      Author: Sano
 */

#include "stm32f4xx_hal.h"
#include "math_operations.h"

#ifndef SENSORS_H_
#define SENSORS_H_

void gyro_start(void);

int gyro_getBuf(int);

void gyro_pushback_data(void);

uint8_t uart_getc_5(void);

uint8_t uart_getc_3(void);

double pixy_getVecSP_x(void);

double pixy_getVecAng(void);

double gyro_getRoll(void);

double gyro_getPitch(void);

double gyro_getYaw(void);

void pixy_update(void);

void gyro_update(void);

#endif /* SENSORS_H_ */
