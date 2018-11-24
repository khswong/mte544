#include <eigen3/Eigen/Dense>
#include <prm.h>

#include "graph.h"

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) 
{

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

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

void PrmPlanner::sample() {
  // While s and g are not connected
  int i = 0;
  while (milestones.isReachable(s, g)) {
    // Choose random point q from map C with prob P
    // If q doesn't intersect with occupancyGrid, it's ok
    if (!checkCollisionMap(q)) {
      Node sample;
      sample.position = q;
      sample.id = ++i;
      milestones.addVertex(sample);
    }
    // For each v in graph, if straight line from q to v
    // is not interrupted by occupancyGrid, add vertex
  }
}

void PrmPlanner::addGoal() {}

bool PrmPlanner::checkCollision(Eigen::Vector3d a, Eigen::Vector3d b) {
  return true;
}
bool PrmPlanner::checkCollisionMap(Eigen::Vector3d a)

void PrmPlanner::setMap(std::vector<int> map_data, int width, int height) {}
