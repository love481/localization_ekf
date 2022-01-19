/*
 *Author:Love Panta
 *Date:28/09/2021
 */
#pragma once
#ifndef _LOCALIZATION_EKF_H
#define _LOCALIZATION_EKF_H
#include<vector>
#include"localization_ekf/linalg.h"
#include<math.h>
using namespace linalg;
using namespace linalg::aliases;
#define ScanNb 5
#define mapLineNb 4
std::vector<std::vector<double>> Map={{0,6.950},{M_PI/2,2.0},{M_PI,0},{-M_PI/2,0}};
struct jac_meas{
   mat<float,2,3>H_;//jacobian of measurement function w.r.t state
   std::vector<std::vector<float>>_H;
};
struct meas_noise{
   mat<float,2,2>R_;//measurement noise covariance
};
struct meas{
vec<float,2>Z;
};
struct EKF_param
{ 
  mat<float,3,3>F_x;//jacobian of motion model w.r.t state
  mat<float,3,3>F_u;//jacobian of motion model w.r.t input
  mat<float,3,3>Q; //system noise covariance
  mat<float,3,3>P;//covariance prediction
  mat<float,3,2>K_gain;//kalman_gain
  //meas_noise R[ScanNb];
  mat<float,2,2>R;
  //jac_meas H[mapLineNb];
  mat<float,2,3>H;
  float g;//validation gate
};
class kalman_filter
{  private:
      EKF_param *param;
      vec<float,3> pose_state;//(x,y,theta)
      vec<float,3>input_state;//(del_x,del_y,del_theta)
      std::vector<std::vector<float>>scan; //(scan_angle,range)
     // meas pred_meas_state_[mapLineNb];//(scan_angle,range)
      vec<float,2>Z_j;
   public:
      kalman_filter();
      kalman_filter(kalman_filter &&) = default;
      kalman_filter(const kalman_filter &) = default;
      kalman_filter &operator=(kalman_filter &&) = default;
      kalman_filter &operator=(const kalman_filter &) = default;
      ~kalman_filter();
      void set_EKF_param(EKF_param *p){param=p;}
      void setRobotPose(std::vector<float>);
      void setRobotInput(std::vector<float>);
      void state_prediction();
      void state_update();
      vec<float,3> transitionFunction(vec<float,3>,vec<float,3>);
      void measurementFunction(vec<float,2>);
      void set_obser_meas_state(std::vector<std::vector<float>>);
      std::vector<float>get_robotPose();
};
//function to fit the best line from the set of (x,y) coordinates
inline std::vector<int>fitLine(std::vector<std::vector<float>>XY)
{ 
  std::vector<float>sum(2);
  std::vector<int>m(2);
  std::vector<int>num_den(2);
  num_den[0]=0;
  num_den[1]=0;
  for( int i =0;i<XY[0].size();i++)
  {
   sum[0]+=XY[0][i];
   sum[1]+=XY[1][i];
  }
  sum[0]=sum[0]/XY[0].size();
  sum[1]=sum[1]/XY[0].size();
  for( int i =0;i<XY[0].size();i++)
  {
    num_den[0]+=(XY[0][i]-sum[0])*(XY[1][i]-sum[1]);
    num_den[1]+=std::pow((XY[1][i]-sum[1]),2)-std::pow((XY[0][i]-sum[0]),2);
   
  }
  num_den[0]*=-2;
  float alpha=std::atan2(num_den[0],num_den[1])/2;
  float r=sum[0]*std::cos(alpha)+sum[1]*std::sin(alpha);
  if (r < 0)
  {
    alpha = alpha + M_PI;
    if (alpha > M_PI)
      alpha = alpha - 2 * M_PI;
    r = -r;
  }
  m[0]=alpha;
  m[1]=r;
  return m;
}
#endif // !_LOCALIZATION_EKF_H