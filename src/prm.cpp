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
  // While s and g are not connected
  int i = Milestones.size();
  // Lavalle's nonuniform sampling algorithm
  // Uniform randomly
  std::uniform_real_distribution<double> x_urd(0.0, (double)width);
  std::uniform_real_distribution<double> y_urd(0.0, (double)height);
  std::normal_distribution<double> l_urd(0.0, l_distribution);
  while (Milestones.isReachable(s, g)) {
    // Choose random point q from map C with prob P
    Eigen::Vector2d q;
    Eigen::Vector2d q_lavalle;
    // Generate one point 
    q << x_urd(generator) << y_urd(generator);
    q_lavalle << q(0) + l_urd(generator) << q(1) + l_urd(generator);
    if (checkCollisionMap(q) xor checkCollisionMap(q_lavalle))
      {
        q = checkCollisionMap(q) ? q_lavalle : q;
      Node sample;
      sample.position = q;
      sample.id = ++i;
      Milestones.addVertex(sample);

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
}

void PrmPlanner::addGoal() {}

bool PrmPlanner::checkCollisionLine(Node a, Node b) {
  return true;
}
bool PrmPlanner::checkCollisionMap(Eigen::Vector2d a) {}

void PrmPlanner::setMap(std::vector<int> map_data, int width, int height) {}
