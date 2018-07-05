// CubicCurve.h
// 三次のパラメトリック関数で曲線を生成します
/*
   クラス：
     CubicCurve

     クラス内関数：
       double x (double u)
         uのときのx座標を求めます。

         double u : 媒介変数
         

       double y (double u)
         uのときのy座標を求めます。

         double u : 媒介変数


       double x_prime (double u)
         uのときのベクトルのx成分を求めます。

         double u : 媒介変数
         

       double y_prime (double u)
         uのときのベクトルのy成分を求めます。
         
         double u : 媒介変数


       double vecAng (double u)
         uのときのベクトルの角度を求めます。
         (-PI<ang<PI)

         double u : 媒介変数


       void bezier (double p[4][2])
         条件を満たす三次のベジェ曲線を生成します。
         (0<=u<=1)

         double p[4][2] : p[4][2] = {{x0, y0}, ... {x3, y3}}
                          制御点となる4点(x0,y0)~(x3,y3)を指すポインタ
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
};

#endif //CUBICCURVE_H
