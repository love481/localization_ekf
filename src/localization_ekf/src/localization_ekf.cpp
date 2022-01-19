#include"localization_ekf/localization_ekf.h"
kalman_filter::kalman_filter()
{
 param=new EKF_param();
 pose_state={0,0,0};
 input_state={0,0,0};
 param->F_x=identity;
 param->F_u=identity;
 param->Q={{abs(input_state[0]),0,0},{0,abs(input_state[1]),0},{0,0,abs(input_state[2])}};
 param->P=float(0.01)*(mat<float,3,3>)identity;
//  for(int i=0;i<ScanNb;i++)
//     {
//      param->R[i].R_=float(0.01)*(mat<float,2,2>)identity;
//      param->H[i].H_={{0,0},{0,0},{0,0}};
//     }
 param->R=float(0.01)*(mat<float,2,2>)identity;
 param->H={{0,0},{0,0},{0,0}};
 param->g=0.01;
 param->K_gain={{0,0,0},{0,0,0}};

}
void kalman_filter::setRobotPose(std::vector<float> pose)
{
  for(int i=0;i<pose.size();i++)
     pose_state[i]=pose[i];
}
void kalman_filter::setRobotInput(std::vector<float> input)
{
  for(int i=0;i<input.size();i++)
     input_state[i]=input[i];
}

vec<float,3>kalman_filter::transitionFunction(vec<float,3>prev_state,vec<float,3>curr_input)
{
  //process model function is defines here
  //prev_state(x_t-1,y_t-1,theta_t-1),curr_input(del_x,del_y,del_theta_fromIMU)
  return prev_state+curr_input;
}
 void kalman_filter:: measurementFunction(vec<float,2>map)
 {
  Z_j={map[0]-pose_state[2],map[1]-(pose_state[0]*cos(map[0])+pose_state[1]*sin(map[0]))};
  param->H={{0,-cos(map[0])},{0,-sin(map[0])},{-1,0}};
 }
void kalman_filter::state_prediction()
{
 pose_state=transitionFunction(pose_state,input_state);
 param->Q={{abs(input_state[0]),0,0},{0,abs(input_state[0]),0},{0,0,abs(input_state[0])}};
 param->P=mul(param->F_x,mul(param->P,transpose(param->F_x)))+mul(param->F_u,mul(param->Q,transpose(param->F_u)));
}
void kalman_filter::set_obser_meas_state(std::vector<std::vector<float>>laserScan)
{ 
    scan.reserve(laserScan.size());
    scan=laserScan;
}
void kalman_filter::state_update()
{
    for(int i=0;i<scan[0].size();i++)
    {
      mat<float,2,1> v=vec<float,2>{scan[i][0],scan[i][1]}-Z_j;
      mat<float,2,2>innov_cov=mul(param->H,mul(param->P,transpose(param->H)))+param->R;
      if((mul(transpose(v),mul(inverse(innov_cov),v)))<=(mat<float,1,1>)param->g)
      {
        param->K_gain=mul(param->P,mul(transpose(param->H),inverse(innov_cov)));
        mat<float,3,1> del_pose=mul(param->K_gain,v);
        pose_state=pose_state+vec<float,3>{del_pose.row(0)[0],del_pose.row(0)[1],del_pose.row(0)[2]};
        param->P=param->P-mul(param->K_gain,mul(innov_cov,transpose(param->K_gain)));
      }
    }

}
std::vector<float>kalman_filter::get_robotPose()
{
  std::vector<float>pose(3);
  for(int i=0;i<3;i++)
     pose[i]=pose_state[i];

  return pose;
}