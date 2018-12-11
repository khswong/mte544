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
  std::vector<Eigen::Vector2d> __milestones;
  int width, height;

 public:
  
  bool checkCollisionLine(Node a, Node b);
  bool checkCollisionMap(Eigen::Vector2d q);
  PrmPlanner(){};
  ~PrmPlanner(){};
  void sampleMilestones();
  void setMap(std::vector<signed char> map_data, int w, int h);
  void setRes(float res);
  void setGoal(Eigen::Vector2d goal);
  void setPos(Eigen::Vector2d pos);
  float getRes() { return Resolution; };
  std::vector<Eigen::Vector2d> getMilestones() {return __milestones;};
  Eigen::MatrixXi getOccupancyMap() {return OccupancyMap;};
  std::vector<Eigen::Vector2d> getPath();


};
#endif
