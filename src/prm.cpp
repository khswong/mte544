#include "prm.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "graph.h"

short sgn(int x) { return x >= 0 ? 1 : -1; }
// Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The
// calculated
//        points (x, y) are stored in the x and y vector. x and y should be
//        empty
//	  vectors of integers and shold be defined where this function is called
// from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x,
               std::vector<int> &y) {

  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int dx2 = x1 - x0;
  int dy2 = y1 - y0;

  const bool s = abs(dy) > abs(dx);

  if (s) {
    int dx2 = dx;
    dx = dy;
    dy = dx2;
  }

  int inc1 = 2 * dy;
  int d = inc1 - dx;
  int inc2 = d - dx;

  x.push_back(x0);
  y.push_back(y0);

  while (x0 != x1 || y0 != y1) {
    if (s)
      y0 += sgn(dy2);
    else
      x0 += sgn(dx2);
    if (d < 0)
      d += inc1;
    else {
      d += inc2;
      if (s)
        x0 += sgn(dx2);
      else
        y0 += sgn(dy2);
    }

    // Add point to vector
    x.push_back(x0);
    y.push_back(y0);
  }
}

#define max_dist 50
#define l_distribution 1
void PrmPlanner::sampleMilestones() {
  int i = Milestones.size() + 1;
  // Lavalle's nonuniform sampling algorithm
  // Uniform randomly
  std::uniform_real_distribution<double> x_urd(0.0, OccupancyMap.rows());
  std::uniform_real_distribution<double> y_urd(0.0, OccupancyMap.cols());
  std::normal_distribution<double> l_nd(0.0, 1);
  // While s and g are not connected
  do {
    // Choose random point q
    Eigen::Vector2d q;
    Eigen::Vector2d q_lavalle;
    // Generate one point
    q << x_urd(generator), y_urd(generator);
    q_lavalle << q(0) + l_nd(generator), q(1) + l_nd(generator);
    if (checkCollisionMap(q) xor checkCollisionMap(q_lavalle)) {
      q = checkCollisionMap(q) ? q_lavalle : q;
      Node sample;
      sample.position = q;
      sample.id = i;
      if (Milestones.addVertex(sample)) {
        i++;
        ROS_INFO("ADD NEW POINTS %d", i);
        ROS_INFO("NEW POINT x %d y %d", sample.position(0), sample.position(1));
        // For each v in graph, if straight line from q to v
        // is not interrupted by occupancyGrid, add vertex
        for (std::map<int, Node>::iterator curstone = Milestones.begin();
             curstone != Milestones.end(); ++curstone) {
          Node curnode = (*curstone).second;
          if ((curnode.position - sample.position).norm() < max_dist &&
              !checkCollisionLine(curnode, sample)) {
            Milestones.addEdge(curnode, sample);
            ROS_INFO("ADD EDGES BETWEEN %d and %d", curnode.id, sample.id);
          }
        }
      }
      else
      {
        ROS_INFO ("Can't add point");
      }
    }
  } while (!Milestones.isReachable(CurPos, Goal));
}

#define thresh 20
bool PrmPlanner::checkCollisionLine(Node a, Node b) {
  std::vector<int> x, y;
  bresenham(a.position(0), a.position(1), b.position(0), b.position(1), x, y);
  std::vector<int>::iterator x_ptr = x.begin(), y_ptr = y.begin();
  while (x_ptr != x.end() || y_ptr != y.end()) {
    int temp_x = (*x_ptr) > OccupancyMap.rows() ? OccupancyMap.rows() : (*x_ptr);
    int temp_y = (*y_ptr) > OccupancyMap.cols() ? OccupancyMap.cols() : (*y_ptr);

    temp_x = temp_x < 0 ? 0 : temp_x;
    temp_y = temp_y < 0 ? 0: temp_y;

    if ( OccupancyMap(temp_x, temp_y) > thresh ) {
      return true;
    }
    x_ptr++;
    y_ptr++;
  }
  return false;
}

// "Monte carlo" collision detection
// Aka the "I've given up and I've already finished the final" algorithm
#define robot_radius 1
#define collision_samples 20
bool PrmPlanner::checkCollisionMap(Eigen::Vector2d a) {
  int x = a(0) > OccupancyMap.rows() ? OccupancyMap.rows()  : a(0);
  int y = a(1) > OccupancyMap.cols() ? OccupancyMap.cols()  : a(1);
  ROS_INFO("Check collision map");
  ROS_INFO("x y %d %d", x, y);
  ROS_INFO("Size of Map x: %d y: %d", OccupancyMap.rows(), OccupancyMap.cols());
  ROS_INFO("What is here at (x,y) %d", OccupancyMap(x, y));
  if (OccupancyMap(x, y) > thresh) {
    std::normal_distribution<double> collision_nd(0.0, robot_radius);
    for (int i = 0; i < collision_samples; i++) {

    int temp_x = x + (int)collision_nd(generator);
    temp_x = (temp_x) > width ? (width) : (temp_x);
    temp_x = (temp_x) < width ? (width) : (temp_x);
    int temp_y = y + (int)collision_nd(generator);
    temp_y = (y) > height? (height) : (y);
    temp_y = (temp_y) < height ? (height) : (temp_y);

    if (OccupancyMap(temp_x, temp_y) > thresh) {
        ROS_INFO("Collision");
        return true;
      }
    }
    ROS_INFO("We ok");
    return false;
  }
}

void PrmPlanner::setMap(std::vector<signed char> map_data, int w, int h) {
  ROS_INFO("w h %d %d", w, h);
  width = w;
  height = h;
  Eigen::MatrixXi mapMatrix(width, height);
  for (std::vector<signed char>::iterator mapIter = map_data.begin();
       mapIter != map_data.begin(); ++mapIter) {
    mapMatrix << (*mapIter);
  }
  OccupancyMap = mapMatrix.transpose();
  ROS_INFO("Size of Map x: %d y: %d", OccupancyMap.rows(), OccupancyMap.cols());
}

void PrmPlanner::setGoal(Eigen::Vector2d goal) {
  ROS_INFO("SetGoal");
  Goal.position = goal;
  Goal.id = Milestones.size() + 1;
  // Try to insert into hash map - if possible, populate edges
  if (Milestones.addVertex(Goal)) {
    for (std::map<int, Node>::iterator curstone = Milestones.begin();
         curstone != Milestones.end(); ++curstone) {
      Node curnode = (*curstone).second;
      if ((curnode.position - Goal.position).norm() < max_dist ) {//&&
        //  !checkCollisionLine(curnode, Goal)) {
        Milestones.addEdge(curnode, Goal);
      }
    }
  }
  sampleMilestones();
}

void PrmPlanner::setPos(Eigen::Vector2d pos) {

  CurPos.position = pos;
  CurPos.id = Milestones.size() + 1;
  //
  if (Milestones.addVertex(CurPos)) {
    for (std::map<int, Node>::iterator curstone = Milestones.begin();
         curstone != Milestones.end(); ++curstone) {

      Node curnode = (*curstone).second;
      if ((curnode.position - CurPos.position).norm() < max_dist ) {
        //&& !checkCollisionLine(curnode, CurPos)) {
        Milestones.addEdge(curnode, CurPos);
      }
    }
  }
}

std::vector<Eigen::Vector2d> PrmPlanner::getPath() {

  std::vector<int> waypoints = Milestones.getPath();
  std::vector<Eigen::Vector2d> path;

  for (std::vector<int>::iterator itr = waypoints.begin(); itr != waypoints.end(); itr++)
  {
    Eigen::Vector2d goal = Milestones.getNode(*itr);
    goal /= Resolution;
    path.push_back(goal);
  }
}

void PrmPlanner::setRes(float res){Resolution = res;}
