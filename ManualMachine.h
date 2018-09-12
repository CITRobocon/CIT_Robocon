#ifndef MANUALMACHINE_H
#define MANUALMACHINE_H

#include "mbed.h"

/*defines*/
/*********************************LED******************************************/

DigitalOut RLED(PC_0),GLED(PC_1),BLED1(PB_0),BLED2(PA_4);   

/*********************************switch***************************************/

DigitalIn TSW(PC_3),PSW(PC_2);

/**********************************sensor**************************************/

DigitalIn PI1(PC_6),PI2(PC_5);

/*********************************motor*****************************************/
/* M：持ち上げモータ S：走行モータ*/

PwmOut servo(PC_8),M_IN1(PC_7),M_IN2(PC_9)
PwmOut S1_PWM(PA_9),S2_PWM(PA_8);
DigitalOut S1_IN1(PC_4),S1_IN2(PB_13),S1_STBY(PB_1);
DigitalOut S2_IN1(PB_5),S2_IN2(PB_4),S2_STBY(PA_10);

/********************************serial****************************************/

#endif //MANUALMACHINE_H
