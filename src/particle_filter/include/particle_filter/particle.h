/*
 *Author:Love Panta
 *Date:20/09/2021
 */
#pragma once
#ifndef PARTICLE_H_
#define PARTICLE_H_
#include<particle_filter/filter_utils.h>
#define WorldSizeX 100
#define WorldSizeY 100
#define mapResolution 0.05
#define originX 0
#define originY 0
double map[WorldSizeX][WorldSizeY];
struct laserScan
{
  double range;
  double scanAngle;
  double scanTime;
};
struct Pose
{
  double x;
  double y;
  double theta;
};
struct particle
{ 
    Pose pose;
    double weight;
};
class particleFilter
{
    private:
      int particlesNbr;
      Pose *robot;
      vector<particle> particles;
      vector<double> correction;
      vector<int>lidar_global;
      int oned_ind(vector<int>);
      vector<int> twod_ind(int ind);
    public:
      particleFilter();
      particleFilter(particleFilter &&) = default;
      particleFilter(const particleFilter &) = default;
      particleFilter &operator=(particleFilter &&) = default;
      particleFilter &operator=(const particleFilter &) = default;
      ~particleFilter(){}
      void setNumOfParticles(int);
      void set_robotPose(double,double,double);
      void update_measurement(vector<laserScan>);
      void set_probabilty(int,vector<int>);
      void resample_particles(void);
      Pose * getRobotPose(void);
};
#endif // !PARTICLE_H_
