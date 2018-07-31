// オドメトリによる仮想経路追従PID,FF制御

#include "mbed.h"
#include "SHIRAGIKU.h"
#include "Odometry.h"
#include "CubicCurve.h"

Odometry odometry(&encR, &encL, 2000, 1.0, ENCODER_RADIUS, ENCODER_DISTANCE, 0.010);

CubicCurve path;

Ticker control;

double u = 0.0;

const double Kp_pos = 0.025, Ki_pos = 0.00010, Kd_pos = 0.000010;
const double Kp_ang = 3.0, Ki_ang = 0.0030, Kd_ang = 0.00030;

double ep_pos, ei_pos = 0.0, ed_pos, ep_pos0 = 0.0;
double ep_ang, ei_ang = 0.0, ed_ang, ep_ang0 = 0.0;

#define period 0.20

double ff;

void handler (void){
    double R = path.curvRad(u);
    
    if (R == 0.0 || isnan(R))
        ff = 0.0;
    else
        ff = 0.50*WHEEL_DISTANCE/2.0/R;
         
    ep_pos = -(path.x(u)-odometry.x)*sin(odometry.ang) + (path.y(u)-odometry.z)*cos(odometry.ang);
    ei_pos += ep_pos*period;
    ed_pos = (ep_pos-ep_pos0)/period;
    ep_pos0 = ep_pos;
    
    if (ei_pos > 1000)
        ei_pos = 1000;
    
    ep_ang = -odometry.ang + path.vecAng(u);
    ei_ang += ep_ang*period;
    ed_ang = (ep_ang-ep_ang0)/period;
    ep_ang0 = ep_ang;
    
    if (ei_pos > 100)
        ei_pos = 100;
    
    float duty = (float)ff;
    duty += (float)(Kp_pos*ep_pos + Ki_pos*ei_pos + Kd_pos*ed_pos);
    duty += (float)(Kp_ang*ep_ang + Ki_ang*ei_ang + Kd_ang*ed_ang);
    
    float duty_r = 0.5f + duty;
    float duty_l = 0.5f - duty;
    
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
    control.attach(&handler, period);
    while(u < 1.0){
        /*
        printf ("p(pos,ang) = (%lf, %lf)\n\r", Kp_pos*ep_pos, Kp_ang*ep_ang);
        printf ("   i(pos,ang) = (%lf, %lf)\n\r", Ki_pos*ei_pos, Ki_ang*ei_ang);
        printf ("       d(pos,ang) = (%lf, %lf)\n\r", Kd_pos*ed_pos, Kd_ang*ed_ang);
        */
        
        printf ("p(pos,ang) = (%lf, %lf)\n\r", ep_pos, ep_ang);
        printf ("   i(pos,ang) = (%lf, %lf)\n\r", ei_pos, ei_ang);
        printf ("       d(pos,ang) = (%lf, %lf)\n\r", ed_pos, ed_ang);
        
        //printf ("x,z,ang = %lf, %lf, %lf\n\r", odometry.x, odometry.z, odometry.ang);
    }
    control.detach();
    m(2.0f,2.0f);
    printf ("finish!\n\r");
}
