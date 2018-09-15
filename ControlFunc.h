#ifndef CONTROLFUNC_H
#define CONTROLFUNC_H

#include "mbed.h"
#include "AutoMachine.h"
#include "Odometry.h"
#include "CubicCurve.h"
#include "Sabertooth232.h"

//for move
float period = 0.050;
float ave_duty, duty_1, duty_2, max_ratio = 1.0;
char div_vel, stage_vel;
float vel_grad[5];
//odometry control
double u = 0.0;
float R;
float ff;
float fbGain_y, fbGain_theta;
float cu_y, cu_theta;
float dtheta;
//state
int dir = 1;
int be_moving;

CubicCurve path;
Ticker control_md;

extern Odometry odometry;

void AdjustDutyRatio (void){    
    if (duty_1 > max_ratio)
      duty_1 = max_ratio;
    if (duty_1 < -max_ratio)
      duty_1 = -max_ratio;
      
    if (duty_2 > max_ratio)
      duty_2 = max_ratio;
    if (duty_2 < -max_ratio)
      duty_2 = -max_ratio;
    
    return;
}

void control_odometry_sfb (void){
    u = path.calcNearestVar(odometry.x,odometry.z,u);
    R = (float)path.curvRad(u);
    
    if (div_vel*u/(stage_vel+1) > 1.0)
        stage_vel++;
        
    ave_duty = (1-div_vel*u+stage_vel)*vel_grad[stage_vel] + (div_vel*u-stage_vel)*vel_grad[stage_vel+1];
    
    if (R == 0.0f || isnan(R))
        ff = 0.0f;
    else
        ff = ave_duty/R*(float)WHEEL_DISTANCE/2.0f;
        
    if (ff > 3.0f*(1-ave_duty)/4.0f){
      ff = 3.0f*(1.0f-ave_duty)/4.0f;
    }
    
    if (dir == 1){
        cu_theta = (odometry.ang-path.vecAng(u));
        cu_y = (-(odometry.x-path.x(u))*sin(path.vecAng(u)) + (odometry.z-path.y(u))*cos(path.vecAng(u)));
    }else if (odometry.ang < 0.0){
        cu_theta = (odometry.ang-path.vecAng(u)+PI);
        cu_y = (-(odometry.x+150*cos(odometry.ang+PI)-path.x(u))*sin(path.vecAng(u)) + (odometry.z+150*sin(odometry.ang+PI)-path.y(u))*cos(path.vecAng(u)));
    }else{
        cu_theta = (odometry.ang-path.vecAng(u)-PI);
        cu_y = (-(odometry.x+150*cos(odometry.ang-PI)-path.x(u))*sin(path.vecAng(u)) + (odometry.z+150*sin(odometry.ang-PI)-path.y(u))*cos(path.vecAng(u)));
    }
    
    if (cu_theta >= (float)PI)
        cu_theta -= 2*PI;
    else if (cu_theta <= -(float)PI)
        cu_theta += 2*PI;
    
    dtheta = -fbGain_y/ave_duty/TOP_SPEED*cu_y - fbGain_theta*cu_theta;
    
    duty_1 = ave_duty + dir*(ff + dtheta*(float)WHEEL_DISTANCE/2.0f/TOP_SPEED);
    duty_2 = ave_duty - dir*(ff + dtheta*(float)WHEEL_DISTANCE/2.0f/TOP_SPEED);
    
    AdjustDutyRatio();
    writeMotor (dir*duty_1, dir*duty_2);
    
    if (u >= 1.0){
        control_md.detach();
        writeMotor(0.0f, 0.0f);
        be_moving = 0;
    }
    
    return;
}

//public-like

void followVirtualLine (double p[4][2], float v0 = NAN, float v1 = NAN, float v2 = NAN, float v3 = NAN, float v4 = NAN){
    while(be_moving){wait(0.010);};
    if (isnan(v0)){
        div_vel = 0;
        vel_grad[0] = 0.5f;
    }else if (isnan(v1)){
        div_vel = 0;
        vel_grad[0] = v0;
    }else if (isnan(v2)){
        div_vel = 1;
        vel_grad[0] = v0;
        vel_grad[1] = v1;
    }else if (isnan(v3)){
        div_vel = 2;
        vel_grad[0] = v0;
        vel_grad[1] = v1;
        vel_grad[2] = v2;
    }else if (isnan(v4)){
        div_vel = 3;
        vel_grad[0] = v0;
        vel_grad[1] = v1;
        vel_grad[2] = v2;
        vel_grad[3] = v3;
    }else{
        div_vel = 4;
        vel_grad[0] = v0;
        vel_grad[1] = v1;
        vel_grad[2] = v2;
        vel_grad[3] = v3;
        vel_grad[4] = v4;
    }
    stage_vel = 0;
    
    u = 0.0;
    path.bezier(p);
    
    if (dir == 1)
        fbGain_y = 10, fbGain_theta = 7;
    else
        fbGain_y = 15, fbGain_theta = 5;
    
    control_md.attach(&control_odometry_sfb, period);
    be_moving = 1;
    
    return;
}

void invert (void){
    dir *= -1;
    return;
}

void setMD_maxRatio (float ratio){
    max_ratio = ratio;
    return;
}

int moving (void){
    if (be_moving)
        return 1;
    else
        return 0;
}

void forciblyStop (void){
    control_md.detach();
    be_moving = 0;
    return;
}

void turnEast (void){
    be_moving = 1;
    writeMotor(-0.6f, 0.6f);
    wait(0.3);
    while (odometry.ang < 0.0){wait(0.010);}
    while (odometry.ang > 0.0){wait(0.010);}
    writeMotor(0.0f, 0.0f);
    be_moving = 0;
    return;
}

void turnWest (void){
    be_moving = 1;
    writeMotor(0.6f, -0.6f);
    wait(0.3);
    while (odometry.ang < 0.0){wait(0.010);}
    while (odometry.ang > 0.0){wait(0.010);}
    writeMotor(0.0f, 0.0f);
    be_moving = 0;
    return;
}


/*for table*/
int onGA (void){
  return (int)(GA.read()*5);
}

void putGA (void){
    servo3.pulsewidth_us(720);
    return;
}

void openTable (void){
    servo3.pulsewidth_us(920);
    return;    
}

void closeTable (void){
    servo3.pulsewidth_us(1950);
    return;        
}


/*for hand*/
void grabA (void){
    servo1.pulsewidth_us(2310);
    return;
}

void releaseA (void){
    servo1.pulsewidth_us(2120);
    return;
}

void lowerHandToReleaseA (void){
    servo2.pulsewidth_us(1050);
    return; 
}

void lowerHandToGrabA (void){
    servo2.pulsewidth_us(1620);
    return; 
}

void raiseHand (void){
    servo2.pulsewidth_us(700);
    return;
}


/*for etc*/
void waitPushASW (void){
    while(!(ASW&1));
    while(ASW&1);
    return;
}

#endif //CONTROLFUNC_H
