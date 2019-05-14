/*
 * sensors.c
 *
 *  Created on: 2019/05/14
 *      Author: Sano
 */

#include "sensors.h"
#include "xprintf.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

volatile double pixy_vec_sp_x, pixy_vec_angle;
volatile double gyro_roll, gyro_pitch, gyro_yaw;

uint8_t uart_getc_5(void)
{
	volatile uint8_t c = 0;
	char buf[1];
	HAL_UART_Receive(&huart5, (uint8_t *)buf, sizeof(buf), 0xFFFF);
	c = buf[0];
	return c;
}

uint8_t uart_getc_3(void)
{
	uint8_t c = 0;
	char buf[1];
	HAL_UART_Receive(&huart3, (uint8_t *)buf, sizeof(buf), 0xFFFF);
	c = buf[0];
	return c;
}

double pixy_getVecSP_x(void){
	return pixy_vec_sp_x;
}

double pixy_getVecAng(void){
	return pixy_vec_angle;
}

double gyro_getRoll(void){
	return gyro_roll;
}

double gyro_getPitch(void){
	return gyro_pitch;
}

double gyro_getYaw(void){
	return gyro_yaw;
}

void pixy_update(void){
	uint8_t data[4] = {};

	/*
	while((data[0] = uart_getc_3()) != 100);
	for (int i = 0; i < 4; i++){
		data[i] = uart_getc_3();
		//xprintf("%d\t", data[i]);
	}
	//xprintf("\n");
	*/


	while ((data[0] = uart_getc_3()) != 100);
	for (int i = 0; i < 4; i++){
		data[i] = uart_getc_3();
	}

	if (data[0] == 0x01)
		pixy_vec_sp_x = 0.0;
	else if (data[0] == 0xFF)
		pixy_vec_sp_x = (double)data[1]-0xFF-1;
	else
		pixy_vec_sp_x = (double)data[1];

	if (data[2] == 0x01)
		pixy_vec_angle = 0.0;
	else if (data[2] == 0xFF)
		pixy_vec_angle = (double)data[3]-0xFF-1;
	else
		pixy_vec_angle = (double)data[3];
}

void gyro_update(void){
	uint8_t data[6] = {};

	while ((data[0] = uart_getc_5()) != 100);
	for (int i = 0; i < 6; i++){
		data[i] = uart_getc_5();
	}

	if (data[0] == 0xFF)
		gyro_roll = (double)data[1]-0xFF-1;
	else
		gyro_roll = (double)data[1];

	if (data[2] == 0xFF)
		gyro_pitch = (double)data[3]-0xFF-1;
	else
		gyro_pitch = (double)data[3];

	if (data[4] == 0xFF)
		gyro_yaw = (double)data[5]-0xFF-1;
	else
		gyro_yaw = (double)data[5];
}

