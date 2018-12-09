// Simple undirected graph class

#include "graph.h"
#include <eigen3/Eigen/Dense>

float calculateCost(Node a, Node b) { return (a.position - b.position).norm(); }

int Graph::size() { return __size; }

bool Graph::addVertex(Node a) {
  if (vertices.insert(std::pair<int, Node>(a.id, a)).second) {
    ++__size;
    return true;
  } else {
    return false;
  }
}

bool Graph::addEdge(Node a, Node b) {
  float cost = calculateCost(a, b);
  if (a.edges.insert(std::pair<int, float>(b.id, cost)).second &&
      b.edges.insert(std::pair<int, float>(a.id, cost)).second) {
    return true;
  } else {
    return false;
  }
}

bool Graph::isReachable(Node a, Node b) {
  std::vector<Node> closedSet;
  std::vector<Node> openSet;
  openSet.push_back(a);
  bool done = false;
  Node currentNode = a;
  while (!openSet.empty() || currentNode == b) {
    
    for (std::vector<Node>::iterator itr = openSet.begin(); itr != openSet.end(); itr++)
      {
      }
  }
  return false;
}

bool Graph::deleteVertex(Node a) { return false; }

bool Graph::deleteEdge(Node a, Node b) { return false; }
