// Simple undirected graph class
#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

#ifndef GRAPH_H
#define GRAPH_H

typedef struct Node {
  int id;
  // X, Y co-odinate
  Eigen::Vector2d position;
  // The map is indexed w/ this pair
  // std::pair<int, float> (id, distance/cost)
  std::map<int, float> edges;
} Node;

class Graph {
  int __size = 0;
  std::vector<int> shortest_path;
  std::map<int, Node> vertices;

public:
  Graph();
  ~Graph();
  int size();
  bool addVertex(Node a);
  bool addEdge(Node a, Node b);
  bool isReachable(Node a, Node b);
  bool deleteVertex(Node a);
  bool deleteEdge(Node a, Node b);
};
#endif
