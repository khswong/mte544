// Simple undirected graph class
#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

#ifndef GRAPH_H
#define GRAPH_H

typedef struct Node
{
  int id;
  Eigen::Vector3d position;
  // std::pair<int, float> (id, distance/cost)
  std::map<int, float> edges;
}Node;


class Graph
{
  std::map<int, Node> vertices;
 public:
  Graph();
  ~Graph();

  bool addVertex(Node a);
  bool addEdge(Node a, Node b);
  bool isReachable(Node a, Node b);
  bool deleteVertex(Node a);
  bool deleteEdge(Node a, Node b);
};
#endif
