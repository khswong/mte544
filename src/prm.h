#include "graph.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#ifndef PRM_H
#define PRM_H
class PrmPlanner {
  float Resolution;
  Eigen::MatrixXi OccupancyMap;
  Graph Milestones;
  Node Goal;
  Node CurPos;
  // Steven Lavalle's sampling stuff
  std::default_random_engine generator;

  void sampleMilestones();
  bool checkCollision();
  bool checkCollisionLine(Node a, Node b);
  bool checkCollisionMap(Eigen::Vector2d q);

public:
  PrmPlanner();
  ~PrmPlanner();
  void setMap(std::vector<int> map_data, int width, int height);
  void setRes(float res);
  void addGoal(Eigen::Vector2d goal);
  void setPos(Eigen::Vector2d pos);
  float getRes() { return Resolution; };
};
#endif
