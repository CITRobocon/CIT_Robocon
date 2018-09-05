#ifndef AUTOMACHINE_H
#define AUTOMACHINE_H

#include "mbed.h"
#include "QEI.h"

/*defines*/
/*********************************LED******************************************/
DigitalOut BLED1(PA_14);
DigitalOut BLED2(PB_15);   
DigitalOut RLED(PC_13);
DigitalOut GLED(PB_7);

/*********************************switch***************************************/
DigitalIn ASW(PC_15);
DigitalIn OSW1(PH_1);
DigitalIn OSW2(PH_0);
DigitalIn OSW3(PC_14);

/**********************************sensor**************************************/
//PhotoReflector
AnalogIn FL(PC_2);
AnalogIn FR(PC_3);
AnalogIn L1(PC_0);
AnalogIn L2(PC_1);
AnalogIn BL(PB_0);
AnalogIn BR(PA_4);
AnalogIn R1(PA_1);
AnalogIn R2(PA_0);

//Encoder
QEI ENC_R (PB_3, PA_2, PA_3, 2000);
QEI ENC_L (PB_5, PB_4, PB_10, 2000);
BusIn qei_in (PB_3, PA_2, PA_3, PB_5, PB_4, PB_10);

/*********************************servo*****************************************/
PwmOut servo1(PA_8);
PwmOut servo2(PC_7);
PwmOut servo3(PB_6);

/********************************serial****************************************/
Serial MD(PA_9,PA_10);

#endif //AUTOMACHINE_H
