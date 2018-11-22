#include <ros/ros.h>
#include <eigen3/Eigen/Dense>


class PrmPlanner
{
  float resolution;
  Eigen::Matrix 
  

public:
  PrmPlanner();
  ~PrmPlanner();
  void setMap( std::vector<int> map_data, int width, int height);
  void setRes( float res);
};
