// CubicCurve.h
// 三次のパラメトリック関数で曲線を生成します

// (7/11) 更新 : CubicCurve クラス内関数 calcNearestVar を追加

/*
   クラス：
     CubicCurve
     
     クラス内関数：
       double x (double u)
       
         uのときのx座標を求めます

         double u : 媒介変数
         
         
       double y (double u)
       
         uのときのy座標を求めます
         
         double u : 媒介変数
         
         
       double x_prime (double u)
       
         uのときのベクトルのx成分を求めます
         
         double u : 媒介変数


       double y_prime (double u)

         uのときのベクトルのy成分を求めます
         
         double u : 媒介変数
       
       
       double vecLen (double u)

         uのときのベクトルの長さを求めます
         
         double u : 媒介変数
       
       
       double vecAng (double u)

         uのときのベクトルの角度を求めます
         (-PI<ang<PI)

         double u : 媒介変数


       void bezier (double p[4][2])

         条件を満たす三次のベジェ曲線を生成します
         (0<=u<=1)

         double p[4][2] : p[4][2] = {{x0, y0}, ... {x3, y3}}
                          制御点となる4点(x0,y0)~(x3,y3)を指すポインタ
                          
                          
       double calcNearestVar (double x, double y, double u)
       
         任意の点の最近点となる曲線上の点の媒介変数をニュートン法により計算します

         double x,y : 任意の点
         double u : 真の値に近い数値
*/

#ifndef CUBICCURVE_H
#define CUBICCURVE_H

#include <math.h>

class CubicCurve{
  double K[2][4];

public:
  double x (double u){
    return K[0][0]*u*u*u + K[0][1]*u*u + K[0][2]*u + K[0][3];
  }

  double y (double u){
    return K[1][0]*u*u*u + K[1][1]*u*u + K[1][2]*u + K[1][3];
  }

  double x_prime (double u){
    return 3.0*K[0][0]*u*u + 2.0*K[0][1]*u + K[0][2];
  }

  double y_prime (double u){
    return 3.0*K[1][0]*u*u + 2.0*K[1][1]*u + K[1][2];
  }
   
  double vecLen (double u){
    return hypot(x_prime(u), y_prime(u));
  }

  double vecAng (double u){
    return atan2(y_prime(u), x_prime(u));
  }

  void bezier (double p[4][2]){
    int i;

    for (i = 0; i < 2; i++){
      K[i][0] = -p[0][i]+3.0*p[1][i]-3.0*p[2][i]+p[3][i];
      K[i][1] = 3.0*p[0][i]-6.0*p[1][i]+3.0*p[2][i];
      K[i][2] = -3.0*p[0][i]+3.0*p[1][i];
      K[i][3] = p[0][i];
    }
  }
  
  double calcNearestVar (double x, double y, double u){
    double num[] = {x, y};
    double Kc[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double ddc;
    int i;
    
    for (i = 0; i < 2; i++){
      Kc[0] += K[i][0]*K[i][0];
      Kc[1] += 2.0*K[i][0]*K[i][1];
      Kc[2] += 2.0*K[i][0]*K[i][2] + K[i][1]*K[i][1];
      Kc[3] += 2.0*K[i][0]*(K[i][3]-num[i]) + 2.0*K[i][1]*K[i][2];
      Kc[4] += 2.0*K[i][1]*(K[i][3]-num[i]) + K[i][2]*K[i][2];
      Kc[5] += 2.0*K[i][2]*(K[i][3]-num[i]);
      Kc[6] += (K[i][3]-num[i])*(K[i][3]-num[i]);
    }
    
    /*Newton's method*/
    for (i = 0; i < 5; i++){
      ddc = (30*Kc[0]*u*u*u*u + 20*Kc[1]*u*u*u + 12*Kc[2]*u*u + 6.0*Kc[3]*u + 2.0*Kc[4]);
      if (ddc != 0.0)
        u -= (6.0*Kc[0]*u*u*u*u*u + 5.0*Kc[1]*u*u*u*u + 4.0*Kc[2]*u*u*u + 3.0*Kc[3]*u*u + 2.0*Kc[4]*u + Kc[5]) / ddc;
    }
    
    return u;
  }
};

#endif //CUBICCURVE_H
