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

#define DATASIZE_GYRO 7
#define DATASIZE_PIXY 5

volatile uint8_t dataBuf_gyro[DATASIZE_GYRO];
volatile double pixy_vec_sp_x = 0.0, pixy_vec_angle = 0.0;
volatile double gyro_roll = 0.0, gyro_pitch = 0.0, gyro_yaw = 0.0;

void gyro_start(void){
	HAL_UART_Receive_DMA(&huart5, (uint8_t *)dataBuf_gyro, DATASIZE_GYRO);
}

int gyro_getBuf (int n){
	return dataBuf_gyro[n];
}

uint8_t uart_getc_5(void)
{
	volatile uint8_t c = 0;
	char buf[1];
	HAL_UART_Receive_DMA(&huart5, (uint8_t *)buf, sizeof(buf));
	c = buf[0];
	return c;
}

uint8_t uart_getc_3(void)
{
	volatile uint8_t c = 0;
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
	int i;
	volatile uint8_t *data_ptr = /*(uint8_t *)*/dataBuf_gyro;
	volatile int sorted_data[DATASIZE_GYRO-1];

	for (i = 0; i < DATASIZE_GYRO; i++){
		if (*data_ptr == 100)
			break;
		data_ptr++;
	}

	for (i = 0; i < DATASIZE_GYRO-1; i++){
		data_ptr++;
		if (data_ptr-dataBuf_gyro >= DATASIZE_GYRO)
			data_ptr = (uint8_t *)dataBuf_gyro;
		sorted_data[i] = *data_ptr;
	}

	if (sorted_data[0] == 0xFF)
		gyro_roll = MATH_DEG_TO_RAD((double)(sorted_data[1]-0xFF-1));
	else
		gyro_roll = MATH_DEG_TO_RAD((double)(sorted_data[1]));

	if (sorted_data[2] == 0xFF)
		gyro_pitch = MATH_DEG_TO_RAD((double)(sorted_data[3]-0xFF-1-17));
	else
		gyro_pitch = MATH_DEG_TO_RAD((double)(sorted_data[3]-17));

	if (sorted_data[4] == 0xFF)
		gyro_yaw = ((double)(sorted_data[5]-0xFF-1));
	else
		gyro_yaw = ((double)(sorted_data[5]));
}

