// Simple undirected graph class

#include "graph.h"
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sstream>

float calculateCost(Node a, Node b) { return (a.position - b.position).norm(); }

int Graph::size() { return vertices.size(); }

bool Graph::addVertex(Node &a) {
  a.id = vertices.size();
  return (vertices.insert(std::pair<int, Node>(a.id, a)).second);
}

bool Graph::addEdge(Node &a, Node &b) {
  float cost = calculateCost(a, b);
  vertices[a.id].edges[b.id] = cost;
  vertices[b.id].edges[a.id] = cost;
  return true;
}

void Graph::deleteVertex(Node &a) {
  for (std::map<int, Node>::iterator itr = vertices.begin();
       itr != vertices.end(); ++itr) {
    deleteEdge((*itr).second, a);
  }
  vertices.erase(a.id);
}

void Graph::deleteEdge(Node &a, Node &b) {
  a.edges.erase(a.edges.find(b.id));
  b.edges.erase(a.edges.find(b.id));
}

std::vector<Eigen::Vector2d>
Graph::reconstruct_path(std::map<int, int> camefrom, int current, int start) {
  std::vector<Eigen::Vector2d> total_path;
  total_path.push_back(vertices[current].position);
  while (camefrom[current]) {
    current = camefrom[current];
    total_path.push_back(vertices[current].position);
  }
  return total_path;
}

std::vector<Eigen::Vector2d> Graph::getPath(Node a, Node b) {
  // ROS_INFO("RUN A*");
  std::vector<int> closedSet;
  std::vector<int> openSet;
  std::map<int, float> fscore;
  std::map<int, float> gscore;
  std::map<int, int> comefrom;
  std::stringstream debug_info;

  shortest_path.clear();
  openSet.push_back(a.id);
  gscore[a.id] = 0;
  fscore[a.id] = (a.position - b.position).norm();

  while (!openSet.empty()) {
    std::sort(openSet.begin(), openSet.end(),
              [&fscore](int a, int b) { return fscore[a] > fscore[b]; });
    int current = openSet.back();
    openSet.pop_back();
    if (current == b.id) {
      return reconstruct_path(comefrom, current, a.id);
    }
    closedSet.push_back(current);
    for (std::map<int, float>::iterator itr = vertices[current].edges.begin();
         itr != vertices[current].edges.end(); ++itr) {
    }
    Node currentNode = vertices[current];
    for (std::map<int, float>::iterator itr = currentNode.edges.begin();
         itr != currentNode.edges.end(); itr++) {
      int neighbour = (*itr).first;
      if (std::find(closedSet.begin(), closedSet.end(), neighbour) ==
          closedSet.end()) {
        float temp_gscore = gscore[current] + currentNode.edges[neighbour];
        if (std::find(openSet.begin(), openSet.end(), neighbour) ==
            openSet.end()) {
          openSet.push_back(neighbour);
        }
        if ((temp_gscore < gscore[neighbour]) || gscore[neighbour] == 0) {
          comefrom[neighbour] = current;
          gscore[neighbour] = temp_gscore;
          fscore[neighbour] =
              gscore[neighbour] +
              (vertices[neighbour].position - vertices[b.id].position)
                  .norm();
        }
      }
    }
  }
  return std::vector<Eigen::Vector2d>();
}

Node Graph::getNode(int id) { return vertices[id]; }

std::map<int, Node>::iterator Graph::begin() { return vertices.begin(); }

std::map<int, Node>::iterator Graph::end() { return vertices.end(); }
