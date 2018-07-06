//　2Sで作成した機体を用いた移動制御の例です  
//  実機への移植が終わり次第削除します

#include "mbed.h"
#include "Motor.h"
#include "Odometry.h"
#include "CubicCurve.h"

QEI encR(GPIO1, GPIO2, NC, 48, QEI::X4_ENCODING);
QEI encL(GPIO3, GPIO4, NC, 48, QEI::X4_ENCODING);
Motor motor_left(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM);	
Motor motor_right(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM);
BusIn in(GPIO1, GPIO2, GPIO3, GPIO4);
BusIn sw(SW1, SW2);

Odometry odometry(&encR, &encL, 48, 0.15f, 0.025f, 0.146f, 0.020f);
CubicCurve path;
Ticker control;

#define T 0.020

#define Kp 2.0
#define Ki 0.020
#define Kd 0.0010

double e;
double ei = 0.0;
double ed;
double e0 = 0.0;

double cost0;

double p[4][2] = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 2.0}};

double i = 0.0;

double x_rel;
double y_rel;


void control_pid (void){
	x_rel = (path.x(i) - odometry.x) * cos(odometry.ang) + (path.y(i) - odometry.z) * sin(odometry.ang);
	y_rel = -(path.x(i) - odometry.x) * sin(odometry.ang) + (path.y(i) - odometry.z) * cos(odometry.ang);
	
	e = atan2(y_rel, x_rel);
	ei += e*T;
	ed = (e - e0)/T;
	e0 = e;
	
	if (ei > 1000)
		ei = 1000;
	else if(ei < -1000)
		ei = -1000;
	
	if (x_rel < 0){
		if (e*Kp + ei*Ki + ed*Kd > 0.25){
			motor_right = motor_right = -1.0;
			motor_left = -0.50;
		}else if (e*Kp + ei*Ki + ed*Kd < -0.25){
			motor_right = -0.50;
			motor_left = -1.0;
		}else{
  		motor_right = -0.75 - (e*Kp + ei*Ki + ed*Kd);
	  	motor_left = -0.75 + (e*Kp + ei*Ki + ed*Kd);
		}
 	}else{
		if (e*Kp + ei*Ki + ed*Kd > 0.25){
			motor_right = 0.5;
			motor_left = 1.0;
		}else if (e*Kp + ei*Ki + ed*Kd < -0.25){
			motor_right = 1.0;
			motor_left = 0.50;
		}else{
  			motor_right = 0.75 -(e*Kp + ei*Ki + ed*Kd);
	  		motor_left = 0.75 + (e*Kp + ei*Ki + ed*Kd);
		}
	}
}
	
	
int main (void){
	in.mode(PullUp);
	sw.mode(PullUp);
	motor_right.setMaxRatio(0.30);
	motor_left.setMaxRatio(0.30);
	
	path.bezier(p);
	
	odometry.setPositon(0.0f, 0.0f, (float)(PI/2.0));
	odometry.start();
	
	control.attach(&control_pid, T);
	
	for (i = 0.050; i <= 1.050; i += 0.050){
		ei = 0.0;
		wait(T);
  		cost0 = x_rel*x_rel+y_rel*y_rel;
		//調整用
		//printf ("(%lf) cost0 = %lf\n\r", i, cost0);
		//printf ("%f, %f\n\r", odometry.x, odometry.z);
		//printf ("%lf, %lf\n\r", path.x(i), path.y(i));
		while (x_rel*x_rel+y_rel*y_rel > 0.10*cost0){
			//調整用
	  		//printf ("	in while : (%lf) path = %lf, %lf\n\r", i, path.x(i), path.y(i));
  			//printf ("	in while : (%lf) state = %f, %f, %f\n\r", i, odometry.x, odometry.z, odometry.ang);
		  	//printf ("	in while : (%lf) cost = %lf, %lf\n\r", i, x_rel,  y_rel);
	  		//printf ("	in while : (%lf) cost = %lf\n\r", i, x_rel*x_rel+y_rel*y_rel);
			//printf (" in while : (%lf) in_theta = %lf\n\r", i, e*Kp + ei*Ki + ed*Kd);
			if ((sw[1]&1) == 0)　break;
		}
		if ((sw[1]&1) == 0)　break;
	}
	control.detach();
	motor_right = 0.0;
	motor_left = 0.0;
	while(1);
}
