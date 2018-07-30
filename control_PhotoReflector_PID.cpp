// フォトリフレクターによる経路追従PID制御

#include "mbed.h"
#include "SHIRAGIKU.h"

//(PR_FR,PR_FL) 同距離で大体0.28~0.29

Ticker control;

float ep, ei = 0.0f, ed, ep0 = 0.0f;
const float Kp = 1.0, Ki = 0.010, Kd = 0.00010;
const float period = 0.020;

void handler (void){
    float duty_r, duty_l;
    
    ep = -PR_FR.read() + PR_FL.read();
    ei += ep*period;
    ed = (ep-ep0)/period;
    ep0 = ep;
    
    if (ei > 1.0f)
      ei = 1.0f;
    
    duty_r = 0.50f + Kp*ep + Ki*ei + Kd*ed;
    duty_l = 0.50f - Kp*ep - Ki*ei - Kd*ed;
        
    if (duty_r > 1.0f){
        duty_r = 1.0;
        duty_l = 0.0;
    }else if (duty_l > 1.0f){
        duty_r = 0.0;
        duty_l = 1.0;
    }
        
    duty_r = duty_r*0.40f + 0.599f;
    duty_l = duty_l*0.40f + 0.599f;
        
    m(duty_l, duty_r);
}
    

int main (void){    
    control.attach(&handler, period);
    
    while(PR_FR.read() > 0.25f || PR_FL.read() > 0.25f)
        printf ("e(p,i,d)  = (%f, %f, %f)\n\r", ep, ei, ed);
        
    control.detach();
    m(2.0f, 2.0f);
    printf ("finish!\n\r");
    while(1);
}
