#include<particle_filter/particle.h>
particleFilter::particleFilter()
{
  particlesNbr=0;
  robot=new Pose();
  robot->x=0;
  robot->y=0;
  robot->theta=0;
  lidar_global.reserve(2);
}
void particleFilter::setNumOfParticles(int n)
{
  particlesNbr=n;
  particles.reserve(particlesNbr);
  correction.reserve(particlesNbr);
}
//change twod index to one dim
int particleFilter:: oned_ind(vector<int> curr)
{
 return curr[0] + WorldSizeY*curr[1];
}
void particleFilter::set_robotPose(double x,double y,double theta)
{
  this->robot->x=x;
  this->robot->y=y;
  this->robot->theta=theta;
  for(int i=0; i<particlesNbr; ++i)
    {
      particles.push_back({gen_gauss_random(robot->x,0.1),gen_gauss_random(robot->y,0.1),gen_gauss_random(robot->theta,0.1),(double)1/particlesNbr});
      correction[i]=0;
    }
}
//change oned index to two dim
vector<int> particleFilter::twod_ind(int ind)
{
	vector<int>twod(2);
	twod[0]= static_cast<int>(ind%WorldSizeY);
	twod[1]= static_cast<int>(floor(ind/ WorldSizeY));
	return twod;
}
//find the probability of each particles based upon the map and range measurement
void particleFilter::set_probabilty(int i,vector<int> lidarGlobal)
{
  // int index=oned_ind({lidarGlobal[1],lidarGlobal[1]});
   if (map[lidarGlobal[1]][lidarGlobal[0]]>0.49)
      correction[i]+=1;
   if (map[lidarGlobal[1]][lidarGlobal[0]]<0.49)
      correction[i]-=1;
   if (correction[i]<0)
      correction[i]=0;
}
void particleFilter::update_measurement(vector<laserScan> scan)
{
    for(int i=0; i<particlesNbr; ++i)
    {
        for(int j=0;j<scan.size();j++)
        {    //Find grid cells hit by the rays (in the grid map coordinate frame)    
            lidar_global[0]=ceil(mapResolution*(scan[j].range*cos(scan[j].scanAngle+particles[i].pose.theta)+particles[i].pose.x))+originX;
            lidar_global[1]=ceil(mapResolution*(-scan[j].range*sin(scan[j].scanAngle+particles[i].pose.theta)+particles[i].pose.x))+originY;
             if (lidar_global[0]>WorldSizeX || lidar_global[0]<=0 ||lidar_global[1]>WorldSizeY || lidar_global[1]<0)
               correction[i]=0;
            else 
            {
              set_probabilty(i,lidar_global);
            }     
        }
    }
}
// resamples the particles and remove the particles having less weight and replace that with of the higher weight
void particleFilter::resample_particles(void)
{
  double sum=0;
  double maxWeight=0;
  int index=0;
  //normalize the weight and assign robot pose the particles having largest weight
  for(int i=0;i<particlesNbr;i++)
  {
   particles[i].weight*=correction[i];
   sum+=particles[i].weight;
  }
  for(int i=0;i<particlesNbr;i++)
  {
   particles[i].weight=particles[i].weight/sum;
   if(particles[i].weight>maxWeight)
      {
        maxWeight=particles[i].weight;
        index=i;
      }
  }
  this->robot=&particles[index].pose;
}
Pose *particleFilter:: getRobotPose(void)
{
 return robot;
}