#include <eigen3/Eigen/Dense>
#include <prm.h>

#include "graph.h"

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

#define max_dist 10
#define l_distribution 1.5
void PrmPlanner::sampleMilestones() {
  int i = Milestones.size();
  // Lavalle's nonuniform sampling algorithm
  // Uniform randomly
  std::uniform_real_distribution<double> x_urd(0.0, (double)width);
  std::uniform_real_distribution<double> y_urd(0.0, (double)height);
  std::normal_distribution<double> l_nd(0.0, l_distribution);
  // While s and g are not connected
  do {
    // Choose random point q
    Eigen::Vector2d q;
    Eigen::Vector2d q_lavalle;
    // Generate one point
    q << x_urd(generator) << y_urd(generator);
    q_lavalle << q(0) + l_nd(generator) << q(1) + l_nd(generator);
    if (checkCollisionMap(q) xor checkCollisionMap(q_lavalle)) {
      q = checkCollisionMap(q) ? q_lavalle : q;
      Node sample;
      sample.position = q;
      sample.id = i;
      if (Milstones.addVertex(sample)) {
        i++;
        // For each v in graph, if straight line from q to v
        // is not interrupted by occupancyGrid, add vertex
        for (std::map<int, Node>::iterator curstone = Milestones.begin();
             curstone != Milestones.end(); ++curstone) {
          if (((*curstone).position - sample.position).norm() < max_dist &&
              !checkCollisionLine(curstone, sample)) {
            Milestones.addEdge((*curstone), sample);
          }
        }
      }
    }
  } while (!Milestones.isReachable(Position, Goal));
}

bool PrmPlanner::checkCollisionLine(Node a, Node b) {
  std::vector<int> x, y;
  bresenham(a.position(0), a.position(1), b.position(0), b.position(1), x, y);
  std::vector<int>::iterator x_ptr = x.begin(), y_ptr = y.begin();
  while (x_ptr != x.end() || y_ptr != y.end()) {
    if (OccupancyMap(x, y) == 1) {
      return true;
    }
  }
  return false;
}

// "Monte carlo" collision detection
// Aka the "I've given up and I've already finished the final" algorithm
#define robot_radius = 3.0;
#define collision_samples 20
bool PrmPlanner::checkCollisionMap(Eigen::Vector2d a) {
  int x = a.position, y = a.position;
  if (!OccupancyMap(x, y)) {
    std::normal_distribution<int> collision_nd(0.0, robot_radius);
    for (int i = 0; i < collision_samples) {
      if (OccupancyMap(x + collision_nd(generator),
                       y + collision_nd(generator))) {
        return true;
      }
    }
    return false;
  }
}

void PrmPlanner::setMap(std::vector<int> map_data, int width, int height) {
  Eigen::MatrixXi mapMatrix(height, width);
  for (std::vector<int>::iterator mapIter = map_data.begin();
       mapIter != map_data.begin(); ++mapIter) {
    mapMatrix << (*mapIter);
  }
  OccupancyMap = mapMatrix.transpose();
}

void PrmPlanner::setGoal(Eigen::Vector2d goal) {
  // 
  Goal.position = goal;
  Goal.id = Milestones.size() + 1;
  // Try to insert into hash map - if possible, populate edges
  if (Milestones.addVertex(Goal)) {
    for (std::map<int, Node>::iterator curstone = Milestones.begin();
         curstone != Milestones.end(); ++curstone) {
      if (((*curstone).position - Goal.position).norm() < max_dist &&
          !checkCollisionLine(curstone, Goal)) {
        Milestones.addEdge((*curstone), Goal);
      }
    }
  }
}

void PrmPlanner::setPos(Eigen::Vector2d pos) {
  CurPos.position = pos;
  sample.id = Milestones.size() + 1;
  //
  if (Milestones.addVertex(CurPos)) {
    for (std::map<int, Node>::iterator curstone = Milestones.begin();
         curstone != Milestones.end(); ++curstone) {
      if (((*curstone).position - CurPos.position).norm() < max_dist &&
          !checkCollisionLine(curstone, CurPos)) {
        Milestones.addEdge((*curstone), CurPos);
      }
    }
  }
}
