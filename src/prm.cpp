#include "prm.h"
#include "graph.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sstream>

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

#define max_dist 20
#define l_distribution 10
#define num_particles 500
void PrmPlanner::sampleMilestones() {
  // Lavalle's nonuniform sampling algorithm
  // Uniform randomly
  std::uniform_real_distribution<double> x_urd(0.0, OccupancyMap.rows());
  std::uniform_real_distribution<double> y_urd(0.0, OccupancyMap.cols());
  std::normal_distribution<double> l_nd(0.0, 1);
  do {
    // Choose random point q
    Eigen::Vector2d q;
    Eigen::Vector2d q_lavalle;
    // Generate one point
    q << x_urd(generator), y_urd(generator);
    q_lavalle << q(0) + l_nd(generator), q(1) + l_nd(generator);
    if (!checkCollisionMap(q)) { // xor checkCollisionMap(q_lavalle)) {
      // q = checkCollisionMap(q) ? q_lavalle : q;
      Node sample;
      sample.position = q;
      if (Milestones.addVertex(sample)) {
        //ROS_INFO("ADD NEW POINTS %d", sample.id);
        //ROS_INFO("NEW POINT x %d y %d", (int)sample.position(0),
        //         (int)sample.position(1));
        // For each v in graph, if straight line from q to v
        // is not interrupted by occupancyGrid, add vertex
        for (std::map<int, Node>::iterator curstone = Milestones.begin();
             curstone != Milestones.end(); ++curstone) {
          Node curnode = (*curstone).second;
          // ROS_INFO("Dist to sample %f", (curnode.position - sample.position).norm() );
          if ((curnode.position - sample.position).norm() < max_dist &&
              !checkCollisionLine(curnode, sample) && curnode.id != sample.id) {
            Milestones.addEdge(curnode, sample);
            if (sample.id == Goal.id)
              {
                ROS_INFO("ADD EDGES BETWEEN %d and %d", curnode.id, sample.id);
              }
          }
        }
      } else {
        ROS_INFO("Can't add point");
      }
    }
  }while(getPath().empty());
  ROS_INFO("done sampling: %d nodes", Milestones.size());
}

#define thresh 90
bool PrmPlanner::checkCollisionLine(Node a, Node b) {
  std::vector<int> x, y;
  bresenham(a.position(0), a.position(1), b.position(0), b.position(1), x, y);
  std::vector<int>::iterator x_ptr = x.begin(), y_ptr = y.begin();
  while (x_ptr != x.end() || y_ptr != y.end()) {
    Eigen::Vector2d point;
    point << *(x_ptr), *(y_ptr);
    if (checkCollisionMap(point)) {
      //ROS_INFO("Line collision");
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
  int x = std::min((int)OccupancyMap.rows() - 1, std::max(0, (int)a(0)));
  int y = std::min((int)OccupancyMap.cols() - 1, std::max(0, (int)a(1)));
  // ROS_INFO("Check collision map");
  // ROS_INFO("x y %d %d", x, y);
  // ROS_INFO("Size of Map x: %d y: %d", OccupancyMap.rows(),
  // OccupancyMap.cols());
  // ROS_INFO("What is here at (%d,%d) %d", x, y, OccupancyMap(x, y));
  if (OccupancyMap(x, y) > thresh)
    {
      // ROS_INFO("Collision at %d %d", x, y);
      return true;
    }
   else{
    std::normal_distribution<double> collision_nd(0.0, robot_radius);
    for (int i = 0; i < collision_samples; i++) {

      int temp_x = x + (int)collision_nd(generator);
      temp_x = std::min((int)OccupancyMap.rows() - 1, std::max(0, (int)temp_x));
      int temp_y = y + (int)collision_nd(generator);
      temp_y = std::min((int)OccupancyMap.rows() - 1, std::max(0, (int)temp_y));
      // ROS_INFO("Check %d %d", temp_x, temp_y);
      if (OccupancyMap(temp_x, temp_y) > thresh) {
        //ROS_INFO("Collision at %d %d", temp_x, temp_y);
        return true;
      }
    }
    //ROS_INFO("We ok");
    return false;
  }
}

void PrmPlanner::setMap(std::vector<signed char> map_data, int w, int h) {
  ROS_INFO("w h %d %d", w, h);
  width = w;
  height = h;
  std::vector<int> map_data_i(map_data.begin(), map_data.end());
  Eigen::MatrixXi mapMatrix(width, height);
  OccupancyMap = Eigen::Map<Eigen::Matrix<int, 100, 100>>(map_data_i.data());
  // ROS_INFO("Size of Map x: %d y: %d", OccupancyMap.rows(),
  // OccupancyMap.cols());
  std::stringstream om_ss;
  om_ss << OccupancyMap;
  //ROS_INFO("OccupancyMap %s", om_ss.str().c_str());
}

void PrmPlanner::setGoal(Eigen::Vector2d goal) {
  Goal.position = goal * (1 / Resolution);
  checkCollisionMap(Goal.position);
  int x  =(int)Goal.position(0);
  int y = (int)Goal.position(1);

  ROS_INFO("What is here at (%d,%d) %d", x, y, OccupancyMap(x, y));
  // Try to insert into hash map - if possible, populate edges
  if (Milestones.addVertex(Goal)) {
    for (std::map<int, Node>::iterator curstone = Milestones.begin();
         curstone != Milestones.end(); ++curstone) {
      Node curnode = (*curstone).second;
      if ((curnode.position - Goal.position).norm() < max_dist &&
          !checkCollisionLine(curnode, Goal)) {
        // ROS_INFO("Add edge to goal %d", Goal.id);
        Milestones.addEdge(curnode, Goal);
      }
    }
  }
  sampleMilestones();
}

void PrmPlanner::setPos(Eigen::Vector2d pos) {

  CurPos.position = pos * (1 / Resolution);
  int x = (int)CurPos.position(0);
  int y = (int)CurPos.position(1);
  ROS_INFO("What is here at (%d,%d) %d", x, y, OccupancyMap(x, y));
  checkCollisionMap(CurPos.position);
  if (Milestones.addVertex(CurPos)) {
    for (std::map<int, Node>::iterator curstone = Milestones.begin();
         curstone != Milestones.end(); ++curstone) {

      Node curnode = (*curstone).second;
      if ((curnode.position - CurPos.position).norm() < max_dist &&
          !checkCollisionLine(curnode, CurPos)) {
        Milestones.addEdge(curnode, CurPos);
      }
    }
  }
}

std::vector<Eigen::Vector2d> PrmPlanner::getPath() {
  std::vector<Eigen::Vector2d> path = Milestones.getPath(CurPos, Goal);
  std::stringstream debug_info;
  debug_info << "Path: ";
  for (std::vector<Eigen::Vector2d>::iterator itr = path.begin(); itr != path.end(); itr++)
    {
      debug_info << " "<< (*itr);
    }
  ROS_INFO("%s", debug_info.str().c_str());
  return Milestones.getPath(CurPos, Goal);
}

void PrmPlanner::setRes(float res) { Resolution = res; }
