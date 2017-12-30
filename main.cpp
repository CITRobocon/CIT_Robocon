//main board test

#include "mbed.h"

PwmOut MD1_1(PA_5),MD1_2(PA_6),MD2_1(PA_7),MD2_2(PB_6); //Motor Driver

DigitalOut led(LED1);

void m(float m1,float m2){ //TB6643KQ MotorDriver PWM function
    float s1,s2,pe;
    float f = 1000;        //frequency(Hz)
    pe =  1/f;
    MD1_1.period(pe);
    MD1_2.period(pe);
    MD2_1.period(pe);
    MD2_2.period(pe);
    s1 = fabsf(m1);
    s2 = fabsf(m2);

    if(0 < m1 && m1 <= 1){
        MD1_1 = s1;
        MD1_2 = 0;
    }else if(m1 < 0){
        MD1_1 = 0;
        MD1_2 = s1;
    }else if(m1 == 2){
        MD1_1 = 1;
        MD1_2 = 1;
    }else{
        MD1_1 = 0;
        MD1_1 = 0;
    }

    if(0 < m2 && m2 <= 1){
        MD2_1 = s2;
        MD2_2 = 0;
    }else if(m2 < 0){
        MD2_1 = 0;
        MD2_2 = s2;
    }else if(m2 == 2){
        MD2_1 = 1;
        MD2_2 = 1;
    }else{
        MD2_1 = 0;
        MD2_1 = 0;
    }
}

int main(void){

    	m(2,2);
    	wait(1);
    	m(1,1);
        wait(1);

    return 0;
}
