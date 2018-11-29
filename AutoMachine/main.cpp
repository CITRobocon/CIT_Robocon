#include "AutoMachine.h"
#include "mbed.h"
#include "Odometry.h"
#include "ControlFunc.h"

Odometry odometry(&ENC_R, &ENC_L, 2000, 1.0, ENCODER_RADIUS, ENCODER_DISTANCE);

extern float duty_1, duty_2;
extern float ave_duty;
extern float ff, cu_y, cu_theta, dtheta;
extern float R;

void stopMD (void){
    writeMotor(0.0f, 0.0f);
    return;
}

void turnGLED (void){
    GLED = OSW2&1;
    return;
}

void reverse (double *p){
    for(int i = 0; i < 2; i++){
        *(p+i) += *(p+6+i);
        *(p+6+i) = *(p+i) - *(p+6+i);
        *(p+i) -= *(p+6+i);
        *(p+2+i) += *(p+4+i);
        *(p+4+i) = *(p+2+i) - *(p+4+i);
        *(p+2+i) -= *(p+4+i);
    }    
}

int main(){
    OSW2_in.rise(&stopMD);
    OSW2_in.rise(&turnGLED);
    OSW2_in.fall(&turnGLED);
    writeMotor(0.0f, 0.0f);
    
    waitPushASW();
    
    //ボタンを押した直後にオドメトリを取り始めるように変更
    odometry.setPosition(206, 206, -PI+0.0000000010);
    odometry.start();
    
    if ((OSW1&1) != 1){
        //init
        grabA();
        raiseHand();
        openTable();
        setMD_maxRatio(1.0f);
    
        while(!onGA()){wait(0.010);}; 
        wait(2.0);
    
        //rotate
        writeMotor(0.4f, -0.4f);
        while (odometry.ang < -PI/2){wait(0.010);}
        writeMotor(0.0f, 0.0f);
    
        wait(0.2);
    
        //to put GA
        invert();
        double p1[4][2] = {{206, 206}, {206, 1435.25}, {930, 584}, {930, 1593}};
        followVirtualLine(p1, 0.7f, 0.65f, 0.5f, 0.5f);
    
        while(moving()){wait(0.010);};
        double p7[4][2] = {{930, 1593}, {930, 1632}, {930, 1674}, {930, 1700}};
        followVirtualLine(p7, 0.5f, 0.8f, 0.8f);
        wait(0.10);
        putGA();
    
        //to get A
        while(moving()){wait(0.010);};
        invert();
    
        double p2[4][2] = {{930, 1700}, {930, 1435.25}, {256, 1435.25}, {256, 2099}};
        followVirtualLine(p2, 0.5f, 0.7f, 0.6f);
        wait(1.0f);
        closeTable();
    
        double p3[4][2] = {{256, 2099}, {256, 2605}, {545, 2605}, {900, 2605}};
        followVirtualLine(p3, 0.6f, 0.7f, 0.7f, 0.3f);
    
        wait(1.0);
        
        releaseA();
        lowerHandToGrabA();
    
    }else{
        releaseA();
        lowerHandToGrabA();
        closeTable();
        setMD_maxRatio(1.0f);
        
        wait(0.5);
        
        odometry.setPosition(206, 2605+104-19, 0.0);
        odometry.start();
        
        double p10[4][2] = {{206, 2605+104}, {400, 2605+104}, {600, 2605}, {900, 2605}};
        followVirtualLine(p10, 0.7f, 0.3f);
    }
    
    wait(0.50);
    while(moving()){
        if (!odometry.moving())
            forciblyStop();
        wait(0.010);
    }
    
    writeMotor(-0.2f, -0.2f);
    wait(0.2);
    writeMotor(0.0f, 0.0f);
    
    grabA();
    
    wait(0.3);
    writeMotor(-0.5f, -0.5f);
    wait(0.1);
    writeMotor(0.0f, 0.0f);
    
    raiseHand();
    
    wait(0.3);
    
    //to put A
    invert();
    
    double p5[4][2] = {{900, 2605}, {580, 2625}, {256, 2605}, {256, 2099}};
    followVirtualLine(p5, 0.5f, 0.6f, 0.6f);
    double p8[4][2] = {{256, 2099}, {256, 1593}, {580, 1593}, {800, 1593}};
    followVirtualLine(p8, 0.6f, 0.6f, 0.5f);
    while(moving()){wait(0.010);}
    turnEast();
    invert();
    double p6[4][2] = {{800, 1593}, {930, 1593}, {1000, 1593}, {1100, 1593}};
    followVirtualLine(p6, 0.5f, 0.5f, 0.3f);
    lowerHandToReleaseA();
    
    wait(0.5);
    while(moving()){
        if (!odometry.moving())
            forciblyStop();
        wait(0.010);
    }
    
    releaseA();
    
    wait(0.3);
    
    //end?
    invert();
    reverse(&p6[0][0]);
    followVirtualLine(p6);
    while(moving()){wait(0.010);}
    turnWest();
    
    invert();
    
    reverse(&p8[0][0]);
    followVirtualLine(p8, 0.5f, 0.6f, 0.6f);
    reverse(&p5[0][0]);
    p5[3][1] += 60;
    followVirtualLine(p5, 0.6f, 0.6f, 0.3f);
    
    wait(1.0);
    releaseA();
    lowerHandToGrabA();
    
    while(moving()){
        if (!odometry.moving())
            forciblyStop();
        wait(0.010);
    }
    
    writeMotor(-0.2f, -0.2f);
    wait(0.2);
    writeMotor(0.0f, 0.0f);
    
    grabA();
    
    wait(0.3);
    writeMotor(-0.5f, -0.5f);
    wait(0.1);
    writeMotor(0.0f, 0.0f);
    
    raiseHand();
    
    wait(0.3);
    
    invert();
    reverse(&p5[0][0]);
    followVirtualLine(p5, 0.5f, 0.6f, 0.6f);
    reverse(&p8[0][0]);
    followVirtualLine(p8, 0.6f, 0.6f, 0.5f);
    
    while(moving()){wait(0.010);}
    turnEast();
    invert();
    reverse(&p6[0][0]);
    followVirtualLine(p6, 0.5f, 0.5f, 0.3f);
    
    lowerHandToReleaseA();
    
    wait(0.5);
    while(moving()){
        if (!odometry.moving())
            forciblyStop();
        wait(0.010);
    }
    
    releaseA();
    
    wait(0.3);
    
    //one more
    invert();
    reverse(&p6[0][0]);
    followVirtualLine(p6);
    while(moving()){wait(0.010);}
    turnWest();
    
    invert();
    
    reverse(&p8[0][0]);
    followVirtualLine(p8, 0.5f, 0.6f, 0.6f);
    reverse(&p5[0][0]);
    p5[3][1] -= 125;
    followVirtualLine(p5, 0.6f, 0.6f, 0.3f);
    
    wait(1.0);
    releaseA();
    lowerHandToGrabA();
    
    while(moving()){
        if (!odometry.moving())
            forciblyStop();
        wait(0.010);
    }
    
    writeMotor(-0.2f, -0.2f);
    wait(0.2);
    writeMotor(0.0f, 0.0f);
    
    grabA();
    
    wait(0.3);
    writeMotor(-0.5f, -0.5f);
    wait(0.1);
    writeMotor(0.0f, 0.0f);
    
    raiseHand();
    
    wait(0.3);
    
    invert();
    reverse(&p5[0][0]);
    followVirtualLine(p5, 0.5f, 0.6f, 0.6f);
    reverse(&p8[0][0]);
    followVirtualLine(p8, 0.6f, 0.6f, 0.5f);
    
    while(moving()){wait(0.010);}
    turnEast();
    invert();
    reverse(&p6[0][0]);
    followVirtualLine(p6, 0.5f, 0.5f, 0.3f);
    
    lowerHandToReleaseA();
    
    wait(0.5);
    while(moving()){
        if (!odometry.moving())
            forciblyStop();
        wait(0.010);
    }
    
    releaseA();
    
    wait(0.3);
    
    invert();
    reverse(&p6[0][0]);
    followVirtualLine(p6);
    
    while(1);
}
