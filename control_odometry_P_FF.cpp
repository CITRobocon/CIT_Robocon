// オドメトリによる仮想経路追従P,FF制御

#include "mbed.h"
#include "SHIRAGIKU.h"
#include "Odometry.h"
#include "CubicCurve.h"

Odometry odometry(&encR, &encL, 2000, 1.0, ENCODER_RADIUS, ENCODER_DISTANCE, 0.010);

CubicCurve path;

Ticker control;

double u = 0.0;

double e_pos;
double e_ang;

void handler (void){
    const double K_pos = 0.25, K_ang = 5.0;
    double R = path.curvRad(u);
    double ff;
    
    if (R == 0.0 || isnan(R))
        ff = 0.0;
    else
        ff = 0.50*WHEEL_DISTANCE/2.0/R;
         
    e_pos = -(path.x(u)-odometry.x)*sin(odometry.ang) + (path.y(u)-odometry.z)*cos(odometry.ang);
    e_ang = -odometry.ang + path.vecAng(u);
    
    float duty_r = (float)(0.50 + ff + K_pos*e_pos + K_ang*e_ang);
    float duty_l = (float)(0.50 - ff - K_pos*e_pos - K_ang*e_ang);
    
    if (duty_r > 1.0f){
        duty_r = 1.0f;
        duty_l = 0.0f;
    }else if (duty_l > 1.0f){
        duty_r = 0.0f;
        duty_l = 1.0f;
    }
    
    duty_r = 0.40f*duty_r + 0.599f;
    duty_l = 0.40f*duty_l + 0.599f;
    
    m(duty_l, duty_r);
    
    u = path.calcNearestVar(odometry.x,odometry.z,u);
}


int main (void){
    double p[4][2] = {{0.0,0.0},{0.0,1000},{1000,1000},{1000,2000}};
    path.bezier(p);
    in.mode(PullUp);
    odometry.setPositon(0.0, 0.0, 3.1415926535/2.0);
    
    odometry.start();
    control.attach(&handler, 0.020f);

    while(u < 1.0)
        printf (" e(pos,ang) = (%lf, %lf)\n\r", e_pos, e_ang);
    
    control.detach();
    m(2.0f,2.0f);
    printf ("finish!\n\r");
}
