#ifndef AUTOMACHINE_H
#define AUTOMACHINE_H

#include "mbed.h"
#include "QEI.h"

/*defines*/
/*********************************machine**************************************/	
#define WHEEL_DISTANCE 272.476
#define WHEEL_RADIUS 28
#define ENCODER_DISTANCE 405.6
#define ENCODER_RADIUS 25.5

/*********************************LED******************************************/
DigitalOut BLED1(PA_14);
DigitalOut BLED2(PA_15);
DigitalOut RLED(PC_13);
DigitalOut GLED(PB_7);

/*********************************switch***************************************/

DigitalIn OSW1(PH_1);
DigitalIn OSW2(PC_12);
DigitalIn OSW3(PA_13);
DigitalIn ASW(PC_10);

/**********************************sensor**************************************/
//PhotoReflector
//     FL FR
//  L1       R1
//  L2       R2
//     BL BR
AnalogIn FL(PC_2);
AnalogIn FR(PC_3);
AnalogIn L1(PC_0);
AnalogIn L2(PC_1);
AnalogIn BL(PB_0);
AnalogIn BR(PA_4);
AnalogIn R1(PA_1);//Unavailable
AnalogIn R2(PA_0);

//GatiasariOkiba
AnalogIn GA(PA_6);

//Encoder
QEI ENC_R (PB_15, PB_1, PB_2, 2000);
QEI ENC_L (PB_5, PB_4, PB_10, 2000);
BusIn qei_in (PB_3, PA_2, PA_3, PB_5, PB_4, PB_10);

/*********************************servo*****************************************/
PwmOut servo1(PA_8);//grab
PwmOut servo2(PC_7);//
PwmOut servo3(PB_6);//table

/********************************serial****************************************/
Serial MD(PA_9,PA_10);


#endif //AUTOMACHINE_H
