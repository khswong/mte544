#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
class ParticleFilter
{
  struct Particle
  {
    double x;
    double y;
    double yaw;
  }

  std::vector<Particle> particles;


}
