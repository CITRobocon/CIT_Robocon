// Odometry.h
// QEI.hを用いてオドメトリを取ります
/*
   クラス：
     Odometry (class QEI *cp1, class QEI *cp2, float PPR, float rario, float radius, float l, float T)

       class QEI *cp1 : 右側エンコーダのQEIクラスポインタ
       class QEI *cp2 : 左側エンコーダのQEIクラスポインタ
       int ppr : 一回転の総パルス数 (pulses per revolution)
       float ratio : ギア比
       float radius : 車輪半径　[m]
       float l : 車輪間距離 [m]
       float T : 計算周期 [s]

    クラス内変数 (public)：
      x : オドメトリにより計算したx座標 [m]
      z : オドメトリにより計算したz座標 [m]
      ang : オドメトリにより計算した向いている方向 (-PI~PI) [rad]

    クラス内関数 (public)：
      void setPosition (float x0, float z0, float ang0)
        入力した位置に設定します

        x0 : x座標
        z0 : z座標
        ang0 : 向いている角度

      void start (void)
        オドメトリの計算を開始します

      void stop (void)
        オドメトリの計算を中断します
        
  #sample#
      QEI encR(PB_15,PB_1,PB_2,1000,QEI::X2_ENCODING);
      QEI encL(PB_3,PB_5,PB_4,1000,QEI::X2_ENCODING);
      BusIn in(PB_3,PB_5,PB_4,PB_15,PB_1,PB_2);

      Odometry odometry(&encR, &encL, 1000, 1.0f, 0.025f, 0.146f, 0.020f);

      int main (void){
         in.mode(PullUp);
         odometry.setPositon(0.0f, 0.0f, 0.0f);
         odometry.start();
         while(1)
            printf("%f, %f, %f\n\r", odometry.x, odometry.z, odometry.ang);
      }
  #sample#
*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "mbed.h"
#include "QEI.h"
#include "math.h"

#define PI 3.14159265358979f

class Odometry{
  float ppr, gear_ratio, wheel_radius, length;
  float period;
  QEI *enc_r, *enc_l;
  Ticker calculate;

  void calcOdometry (void){
    float dr = (float)(enc_r->getPulses())/ppr*gear_ratio*2.0f*PI*wheel_radius;
    float dl = -(float)(enc_l->getPulses())/ppr*gear_ratio*2.0f*PI*wheel_radius;
    float dtheta = (dr-dl)/length;

    x += 0.5f*(dr+dl)*cosf(ang+dtheta/2.0f);
    z += 0.5f*(dr+dl)*sinf(ang+dtheta/2.0f);
    ang += dtheta;
    if (ang < -PI)
     ang += 2.0f*PI;
    else if (ang > PI)
     ang -= 2.0f*PI;

    enc_r->reset();
    enc_l->reset();
  }

public:
  float x, z, ang;

  Odometry (class QEI *qei1, class QEI *qei2, float PPR, float ratio, float radius, float l, float T){
    enc_r = qei1;
    enc_l = qei2;
    gear_ratio = ratio;
    wheel_radius = radius;
    length = l;
    ppr = PPR;
    period = T;
  }

  void setPositon (float x0, float z0, float ang0){
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
