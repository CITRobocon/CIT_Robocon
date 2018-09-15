#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "mbed.h"
#include "QEI.h"

extern int dir;

class Odometry{
   double ppr, gear_ratio, wheel_radius, length;
   double period;
   QEI *enc_r, *enc_l;
   Ticker calculate;
   bool be_moving;

   void calcOdometry (void){
      double dr = -(double)(enc_r->getPulses())/ppr*gear_ratio*2.0*PI*wheel_radius;
      double dl = (double)(enc_l->getPulses())/ppr*gear_ratio*2.0*PI*wheel_radius;
      double dtheta = (dr-dl)/length;

      if (dr == 0 && dl == 0)
         be_moving = false;
      else
         be_moving = true;

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

   Odometry (QEI *qei1, QEI *qei2, double PPR, double ratio, double radius, double l, double T=0.010){
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
   
   bool moving (void){
      return  be_moving;
   }
};

#endif //ODOMETRY_H
