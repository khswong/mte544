// Simple undirected graph class

#include "graph.h"
#include <eigen3/Eigen/Dense>

float calculateCost(Node a, Node b)
{
  return (a.position - b.position).norm();
}

bool Graph::addVertex(Node a)
{
  return vertices.insert(std::pair<int, Node>(a.id, a)).second;
}

bool Graph::addEdge(Node a, Node b)
{
  float cost = calculateCost(a, b);
  if (a.edges.insert(std::pair<int, float>(b.id, cost)).second &&
      b.edges.insert(std::pair<int, float>(a.id, cost)).second )
    {
      return true;
    }
  else
    {
      return false;
    }
}
bool Graph::isReachable(Node a, Node b)
{
  return false;
}
bool Graph::deleteVertex(Node a)
{
  return false;
}

bool Graph::deleteEdge(Node a, Node b)
{
  return false;
}
