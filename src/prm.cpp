#include <prm.h>
#include <eigen3/Eigen/Dense>

#include "graph.h"
void PrmPlanner::sample()
{
  // While s and g are not connected
  while (milestones.isReachable(s, g))
    {
    // Choose random point q from map C with prob P
      
    // If q doesn't intersect with occupancyGrid, it's ok
      if (checkCollision(q, map))
        {
          
        }
    // For each v in graph, if straight line from q to v
    // is not interrupted by occupancyGrid, add vertex

    }
}

void PrmPlanner::addGoal()
{
  
}

void PrmPlanner::checkCollision(Eigen::Vector3d a, Eigen::Vector3d b)
{
  
}

void PrmPlanner::setMap( std::vector<int> map_data, int width, int height)
{
  
}
