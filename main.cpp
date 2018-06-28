//SHIRAGIKU
#include "mbed.h"
#include "QEI.h"

DigitalOut LN(LED1);//NucleoLED

BusOut L(PA_6,PC_4,PB_14);//メイン基板LED

AnalogIn PR_FR(PC_0),PR_FL(PB_0),PR_RR(PC_2),PR_RL(PC_3),PR_FC(PC_1);//フォトリフレクタアナログ読み取り
DigitalIn PR_FLL(PA_4),PR_FRR(PA_1),PR_RC(PB_7),PR_RRR(PC_12),PR_RLL(PC_10);//フォトリフレクタデジタル読み取り

DigitalIn SW(PB_13); //メイン基板タクトスイッチ

DigitalIn EN_L(PC_6),EN_R(PC_8); //車輪1相エンコーダ

PwmOut MD1_1(PA_8),MD1_2(PA_9),MD2_1(PA_10),MD2_2(PA_11); //モータードライバー信号

BusIn in(PB_3,PB_5,PB_4,PB_15,PB_1,PB_2); //エンコーダプルアップ設定用
QEI E_L(PB_3,PB_5,PB_4,1000,QEI::X2_ENCODING); //左接地エンコーダ
QEI E_R(PB_15,PB_1,PB_2,1000,QEI::X2_ENCODING); //右接地エンコーダ


/*Line sensor All digital
DigitalIn PR_FR(PC_0),PR_FL(PB_0),PR_RR(PC_2),PR_RL(PC_3),PR_FC(PC_1),PR_FLL(PA_4),PR_FRR(PA_1),PR_RC(PB_7),PR_RRR(PC_12),PR_RLL(PC_10),
*/


void m(float m1,float m2){ //TB6643KQ モータードライバ関数

    float s1,s2,pe;
    float f = 20000;        //frequency(Hz)
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

#if 1 //エンコーダカウント表示

int main(void){
    in.mode(PullUp);

    while(1){
        int count_l = E_L.getPulses();
        int count_r = E_R.getPulses();
        printf("%d %d\r\n",count_l,count_r);
        wait(0.01);
    }
}

#endif

#if 0
//フォトリフレクタテスト
int main(void){

    while(1){
        printf("F %d | %f | %d | %f | %d || R  %d | %f | %d | %f | %d\r\n"
        ,PR_FLL.read(),PR_FL.read(),PR_FC.read(),PR_FR.read(),PR_FRR.read()
        ,PR_RLL.read(),PR_RL.read(),PR_RC.read(),PR_RR.read(),PR_RRR.read());

    }
}
#endif
