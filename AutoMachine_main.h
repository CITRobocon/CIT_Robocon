#include"mbed.h"

/*defines*/
/*********************************LED******************************************/
DigitalOut BLED1(PH_1);
DigitalOut BLED2(PH_0);
DigitalOut RLED(PC_15);
DigitalOut GLED(PC_14);

/*********************************switch***************************************/
DigitalIn SW(PC_13);
DigitalIn ASW(PB_7);
DigitalIn OSW1(PC_13);
DigitalIn OSW2(PB_15);
DigitalIn OSW3(PA_14);

/**********************************sensor**************************************/
AnalogIn FL(PC_2);
AnalogIn FR(PC_3);
AnalogIn L1(PC_0);
AnalogIn L2(PC_1);
AnalogIn BL(PB_0);
AnalogIn BR(PA_4);
AnalogIn R1(PA_1);
AnalogIn R2(PA_0);

/*********************************servo*****************************************/
PwmOut servo1(PA_8);
PwmOut servo2(PC_7);
PwmOut servo3(PB_6);

/********************************serial****************************************/
Serial MD(PA_9,PA_10);


int main(){
    
}
