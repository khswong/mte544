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
  std::default_random_engine generator;

  int width, height;

  bool checkCollision();
  bool checkCollisionLine(Node a, Node b);
  bool checkCollisionMap(Eigen::Vector2d q);

public:
  PrmPlanner(){};
  ~PrmPlanner(){};
  void sampleMilestones();
  void setMap(std::vector<signed char> map_data, int w, int h);
  void setRes(float res);
  void setGoal(Eigen::Vector2d goal);
  void setPos(Eigen::Vector2d pos);
  float getRes() { return Resolution; };
  std::vector<Eigen::Vector2d> getPath();


};
#endif
