#include "AutoMachine.h"
#include "mbed.h"
#include "Odometry.h"

Serial PC(USBTX, USBRX);

Odometry odometry(&ENC_R, &ENC_L, 2000, 1.0, 25.5, 405.6, 0.010);

int main(){
    qei_in.mode(PullUp);
    while(1){
        servo1 = 0.50;
        servo2 = 0.50;
        servo3 = 0.50;
        PC.printf ("enc(r,l) = (%d, %d)\n\r", ENC_R.getPulses(), ENC_L.getPulses());
        PC.printf (" (x,y) = (%lf, %lf)\n\r", odometry.x, odometry.z);
    }
}
