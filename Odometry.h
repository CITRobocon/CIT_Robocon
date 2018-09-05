// Odometry.h
// QEI.hを用いてオドメトリを取ります
/*
   クラス：
     Odometry (class QEI *cp1, class QEI *cp2, double PPR, double rario, double radius, double l, double T)

       class QEI *cp1 : 右側エンコーダのQEIクラスポインタ
       class QEI *cp2 : 左側エンコーダのQEIクラスポインタ
       double PPR : 一回転の総パルス数 (pulses per revolution)
       double ratio : ギア比
       double radius : 車輪半径
       double l : 車輪間距離
       double T : 計算周期 [s]

    クラス内変数 (public)：
      double x : オドメトリにより計算したx座標
      double z : オドメトリにより計算したz座標
      double ang : オドメトリにより計算した向いている方向 (-PI~PI) [rad]

    クラス内関数 (public)：
      void setPosition (double x0, double z0, double ang0)
        入力した位置に設定します

        double x0 : x座標
        double z0 : z座標
        double ang0 : 向いている角度

      void start (void)
        オドメトリの計算を開始します

      void stop (void)
        オドメトリの計算を中断します
        
   #sample#
     QEI encR(GPIO1,GPIO2,NC,48,QEI::X4_ENCODING);
     QEI encL(GPIO3,GPIO4,NC,48,QEI::X4_ENCODING);
     BusIn in(GPIO1,GPIO2,GPIO3,GPIO4);

     Odometry odometry(&encR, &encL, 48, 0.15, 0.025, 0.146, 0.020);

     int main (void){
        in.mode(PullUp);
        odometry.setPositon(0.0, 0.0, 0.0);
        odometry.start();
        while(1)
           printf("%lf, %lf, %lf\n\r", odometry.x, odometry.z, odometry.ang);
     }
   #sample#
*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "mbed.h"
#include "QEI.h"
#include "math.h"

#define PI 3.14159265358979

class Odometry{
   double ppr, gear_ratio, wheel_radius, length;
   double period;
   QEI *enc_r, *enc_l;
   Ticker calculate;

   void calcOdometry (void){
      double dr = -(double)(enc_r->getPulses())/ppr*gear_ratio*2.0*PI*wheel_radius;
      double dl = (double)(enc_l->getPulses())/ppr*gear_ratio*2.0*PI*wheel_radius;
      double dtheta = (dr-dl)/length;

      x += 0.50*(dr+dl)*cos(ang+dtheta/2.0);
      z += 0.50*(dr+dl)*sin(ang+dtheta/2.0);
    
      ang += dtheta;
      if (ang < -PI)
         ang += 2.0*PI;
      else if (ang > PI)
         ang -= 2.0*PI;

      enc_r->reset();
      enc_l->reset();
   }

public:
   double x, z, ang;

   Odometry (QEI *qei1, QEI *qei2, double PPR, double ratio, double radius, double l, double T){
      enc_r = qei1;
      enc_l = qei2;
      gear_ratio = ratio;
      wheel_radius = radius;
      length = l;
      ppr = PPR;
      period = T;
   }

   void setPosition (double x0, double z0, double ang0){
      x = x0;
      z = z0;
      ang = ang0;
   }

   void start (void){
      enc_r->reset();
      enc_l->reset();
      calculate.attach(this, &Odometry::calcOdometry, period);
   }

   void stop (void){
       calculate.detach();
   }
};

#endif //ODOMETRY_H
