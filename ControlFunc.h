// ControlFunc.h
/*
    class ControlFunc:
        Public functions:
            void followBrightLine (void)
                フォトリフレクタを用いて白線を辿ります
                
            void followVirtualLine (double p[4][2], Odometry *odo)
                オドメトリを用いて仮想の線を辿ります
                
                double p[4][2] : ベジェ曲線の制御点{{x0, y0}, ...{x3, y3}}
                Odometry *odo : Odometryクラスポインタ
                
            void invert (void)
                進行方向を反転させます
                
    #sample#
        #include "ControlFunc.h"

        int main (void){
            Odometry odometry(&encR, &encL, 2000, 1.0, ENCODER_RADIUS, ENCODER_DISTANCE, 0.010);
            ControlFunc controller;
    
            odometry.setPosition(0.0, 0.0, PI/2.0);
            odometry.start();
    
            double p[4][2] = {{0.0, 0.0}, {0.0, 1000}, {1000, 1000}, {1000, 2000}};
    
            controller.followVirtualLine(p, &odometry);
            controller.invert();
            controller.followBrightLine();
    
            printf ("FINISH!\n\r");
            while(1);
        }
    #sample#
*/

#include "mbed.h"
#include "SHIRAGIKU.h"
#include "Odometry.h"
#include "CubicCurve.h"

class ControlFunc {
    double Kp_1, Ki_1, Kd_1, Kp_2, Ki_2, Kd_2;
    double ep_1, ei_1, ed_1, ep_2, ei_2, ed_2, ep0_1, ep0_2;
    double period;
    float duty_1, duty_2;
    double u;
    int dir;
    CubicCurve path;
    Odometry *odometry;
    Ticker control;
    
    double calcPID (void){
        return Kp_1*ep_1 + Ki_1*ei_1 + Kd_1*ed_1 + Kp_2*ep_2 + Ki_2*ei_2 + Kd_2*Kd_2;
    }
    
    void AdjustDutyRatio (void){
        if (duty_1 > 1.0f){
            duty_1 = 1.0f;
            duty_2 = 0.0f;
        }else if (duty_2 > 1.0f){
            duty_1 = 0.0f;
            duty_2 = 1.0f;
        }
        
        duty_1 = duty_1*0.4f + 0.599f;
        duty_2 = duty_2*0.4f + 0.599f;
    }
    
    void control_PR (void){
        if (dir == 1)
            ep_1 = -PR_FR.read() + PR_FL.read();
        else
            ep_1 = PR_RR.read() - PR_RL.read();
        ei_1 += ep_1*period;
        ed_1 = (ep_1-ep0_1)/period;
        ep0_1 = ep_1;
        if (ei_1 > 1.0f)
            ei_1 = 1.0f;
        
        duty_1 = 0.5f + calcPID();
        duty_2 = 0.5f - calcPID();
        
        AdjustDutyRatio();
        
        m(dir*duty_2, dir*duty_1);
    }
    
    void control_odometry (void){
        double R = path.curvRad(u);
        double ff;
    
        if (R == 0.0 || isnan(R))
            ff = 0.0;
        else
            ff = 0.50*WHEEL_DISTANCE/2.0/R;
        
        ep_1 = dir*(-(path.x(u)-odometry->x)*sin(odometry->ang) + (path.y(u)-odometry->z)*cos(odometry->ang));
        ei_1 += ep_1*period;
        ed_1 = (ep_1-ep0_1)/period;
        ep0_1 = ep_1;
        if (ei_1 > 1000)
            ei_1 = 1000;
    
        ep_2 = dir*(-odometry->ang + path.vecAng(u));
        ei_2 += ep_2*period;
        ed_2 = (ep_2-ep0_2)/period;
        ep0_2 = ep_2;
        if (ei_2 > 100)
            ei_2 = 100;
            
        duty_1 = 0.5f + ff + calcPID();
        duty_2 = 0.5f - ff - calcPID();
        
        AdjustDutyRatio();
        
        m(dir*duty_2, dir*duty_1);
    
        u = path.calcNearestVar(odometry->x,odometry->z,u);
    }
    
public:
    ControlFunc (void){
        period = 0.020;
        u = 0.0;
        dir = 1;
    }

    void followBrightLine (void){
        Kp_1 = 1.0, Ki_1 = 0.010, Kd_1 = 0.00010;
        Kp_2 = 0.0, Ki_2 = 0.0, Kd_2 = 0.0;
        ei_1 = 0.0;
        ep0_1 = 0.0;
        
        control.attach(this, &ControlFunc::control_PR, period);
        
        if (dir == 1){
            while (!PR_FRR);
            while (PR_FRR);
        }else{
            while (!PR_RRR);
            while (PR_RRR);
        }
        
        control.detach();
        m(2.0f, 2.0f);
    }
    
    void followVirtualLine (double p[4][2], Odometry *od){
        Kp_1 = 0.025, Ki_1 = 0.00010, Kd_1 = 0.000010;
        Kp_2 = 3.0, Ki_2 = 0.0030, Kd_2 = 0.00030;
        ei_1 = 0.0;
        ei_2 = 0.0;
        ep0_1 = 0.0;
        ep0_2 = 0.0;
        u = 0.0;
        
        odometry = od;
        path.bezier(p);
        
        control.attach(this, &ControlFunc::control_odometry, period);
        
        while(u < 1.0);
        
        control.detach();
        m(2.0f,2.0f);
    }
    
    void invert (void){
        dir *= -1;
    }
};
