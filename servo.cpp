#include "mbed.h"

PwmOut m(D3);
Serial pc(USBTX,USBRX);

int main(){
	m.period_ms(10);
	int pw=1000;
    while(1){

       char input = pc.getc();
       if(input == 'w'){
    	   pw+=30;
       }else if(input == 's'){
    	   pw-=30;
       }
       printf("%d\r\n",pw);

       m.pulsewidth_us(pw);
       
    }
        
        
}
