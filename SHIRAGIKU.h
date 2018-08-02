//SHIRAGIKU
#include "mbed.h"
#include "QEI.h"

#define WHEEL_DISTANCE 424
#define WHEEL_RADIUS 42.5
#define ENCODER_DISTANCE 619
#define ENCODER_RADIUS 25.5

DigitalOut LED_N(LED1);//NucleoLED

BusOut LED(PA_6,PC_4,PB_14);//メイン基板LED

AnalogIn PR_FR(PC_0),PR_FL(PB_0),PR_RR(PC_2),PR_RL(PC_3),PR_FC(PC_1);//フォトリフレクタアナログ読み取り
DigitalIn PR_FLL(PA_4),PR_FRR(PA_1),PR_RC(PB_7),PR_RRR(PC_12),PR_RLL(PC_10);//フォトリフレクタデジタル読み取り

DigitalIn SW(PB_13); //メイン基板タクトスイッチ

DigitalIn EN_L(PC_6),EN_R(PC_8); //車輪1相エンコーダ

PwmOut MD1_1(PA_8),MD1_2(PA_9),MD2_1(PA_10),MD2_2(PA_11); //モータードライバー信号

BusIn in(PB_3,PB_5,PB_4,PB_15,PB_1,PB_2); //エンコーダプルアップ設定用
QEI encL(PB_3,PB_5,PB_4,1000,QEI::X2_ENCODING); //左接地エンコーダ
QEI encR(PB_15,PB_1,PB_2,1000,QEI::X2_ENCODING); //右接地エンコーダ

/*Line sensor All digital
DigitalIn PR_FR(PC_0),PR_FL(PB_0),PR_RR(PC_2),PR_RL(PC_3),PR_FC(PC_1),PR_FLL(PA_4),PR_FRR(PA_1),PR_RC(PB_7),PR_RRR(PC_12),PR_RLL(PC_10),
*/

void m(float m1,float m2){ //TB6643KQ モータードライバ関数
/*
使用方法
m(左モータのデューティー比,右モータのデューティー比)
2でブレーキ
例m(2,2)で両輪ブレーキ
*/
    float s1,s2,pe;
    float f = 20000;    //PWM周波数(Hz)
    pe =  1/f;
    MD1_1.period(pe);
    MD1_2.period(pe);
    MD2_1.period(pe);
    MD2_2.period(pe);
    s1 = fabsf(m1);
    s2 = fabsf(m2);

    if(0 < m1 && m1 <= 1){
        MD1_1 = 0;
        MD1_2 = s1;
    }else if(m1 < 0){
        MD1_1 = s1;
        MD1_2 = 0;
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
