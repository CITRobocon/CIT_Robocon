/*
 * sensors.c
 *
 *  Created on: 2019/05/14
 *      Author: Sano
 */

#include "stm32f4xx_hal.h"
#include "sensors.h"
#include "xprintf.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define BUFSIZE_GYRO 8
#define DATASIZE_GYRO 5

#define BUFSIZE_PIXY 5

volatile uint8_t dataBuf_gyro[BUFSIZE_GYRO];
volatile double pixy_vec_sx_data[3] = {}, pixy_vec_ang_data[3] = {};
volatile double pixy_vec_sx = 0.0, pixy_vec_ang = 0.0;
volatile double gyro_roll_data[DATASIZE_GYRO] = {}, gyro_pitch_data[DATASIZE_GYRO] = {}, gyro_yaw_data[DATASIZE_GYRO] = {};
volatile double gyro_roll = 0.0, gyro_pitch = 0.0, gyro_yaw = 0.0;

double median_filter (double *dataPtr, const int data_size){
	double sorted_data[data_size];
	int i, j;

	for (i = 0; i < data_size; i++){
		sorted_data[i] = *(dataPtr+i);
	}

	for (i = 0; i < data_size-1; i++){
		for (j = 1; j < data_size; j++){
			if (sorted_data[i] > sorted_data[j]){
				sorted_data[i] += sorted_data[j];
				sorted_data[j] = sorted_data[i] - sorted_data[j];
				sorted_data[i] -= sorted_data[j];
			}
		}
	}

	return sorted_data[data_size/2+1];
}

void gyro_start(void){
	const int datasize = 5;

	HAL_UART_Receive_DMA(&huart5, (uint8_t *)dataBuf_gyro, BUFSIZE_GYRO);

	HAL_Delay(1000);
	for (int i = 0; i < datasize; i++){
		gyro_update();
		HAL_Delay(10);
	}
}

int gyro_getBuf (int n){
	return dataBuf_gyro[n];
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
	return pixy_vec_sx;
}

double pixy_getVecAng(void){
	return pixy_vec_ang;
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
		pixy_vec_sx = 0.0;
	else if (data[0] == 0xFF)
		pixy_vec_sx = (double)data[1]-0xFF-1;
	else
		pixy_vec_sx = (double)data[1];

	if (data[2] == 0x01)
		pixy_vec_ang = 0.0;
	else if (data[2] == 0xFF)
		pixy_vec_ang = (double)data[3]-0xFF-1;
	else
		pixy_vec_ang = (double)data[3];
}

void gyro_update(void){
	int i, sorted_data[BUFSIZE_GYRO-2];
	uint8_t *data_ptr = (uint8_t *)dataBuf_gyro;

	for (i = 0; i < BUFSIZE_GYRO-1; i++){
		if (*data_ptr == 100 && *(data_ptr+1) == 100)
			break;
		data_ptr++;
	}
	data_ptr++;

	for (i = 0; i < BUFSIZE_GYRO-2; i++){
		data_ptr++;
		if (data_ptr-dataBuf_gyro == BUFSIZE_GYRO)
			data_ptr = (uint8_t *)dataBuf_gyro;
		else if (data_ptr-dataBuf_gyro > BUFSIZE_GYRO)
			data_ptr = (uint8_t *)dataBuf_gyro+1;
		sorted_data[i] = *data_ptr;
	}

	for (i = 0; i < DATASIZE_GYRO-1; i++){
		gyro_roll_data[i] = gyro_roll_data[i+1];
		gyro_pitch_data[i] = gyro_pitch_data[i+1];
		gyro_yaw_data[i] = gyro_yaw_data[i+1];
	}

	if (sorted_data[0] == 0xFF)
		gyro_roll_data[DATASIZE_GYRO-1] = MATH_DEG_TO_RAD((double)(sorted_data[1]-0xFF-1));
	else
		gyro_roll_data[DATASIZE_GYRO-1] = MATH_DEG_TO_RAD((double)(sorted_data[1]));

	if (sorted_data[2] == 0xFF)
		gyro_pitch_data[DATASIZE_GYRO-1] = MATH_DEG_TO_RAD((double)(sorted_data[3]-0xFF-1-17));
	else
		gyro_pitch_data[DATASIZE_GYRO-1] = MATH_DEG_TO_RAD((double)(sorted_data[3]-17));

	if (sorted_data[4] == 0xFF)
		gyro_yaw_data[DATASIZE_GYRO-1] = MATH_DEG_TO_RAD((double)(sorted_data[5]-0xFF-1));
	else
		gyro_yaw_data[DATASIZE_GYRO-1] = MATH_DEG_TO_RAD((double)(sorted_data[5]));

	gyro_roll = median_filter((double*)gyro_roll_data, DATASIZE_GYRO);
	gyro_pitch = median_filter((double*)gyro_pitch_data, DATASIZE_GYRO);
	gyro_yaw = median_filter((double*)gyro_yaw_data, DATASIZE_GYRO);
}

