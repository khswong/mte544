#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "graph.h"

#ifndef PRM_H
#define PRM_H
class PrmPlanner
{
  float Resolution;
  Eigen::Matrix OccupancyMap;
  Graph Milestones;
  Node Goal;

  void sample();
  bool checkCollision();
  bool checkCollisionMap();
public:
  PrmPlanner();
  ~PrmPlanner();
  void setMap( std::vector<int> map_data, int width, int height);
  void setRes( float res);
  void addGoal( Node goal);
};
#endif
