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
  //(a.edges.insert(std::pair<int, float>(b.id, cost)).second &&
  // b.edges.insert(std::pair<int, float>(a.id, cost)).second);
  // ROS_INFO("ADD EDGES BETWEEN %d and %d with cost %f", a.id, b.id, cost);
  if (a.id == 0 || b.id == 0) {
    //ROS_INFO("Wat %d %f", a.id, a.edges[b.id]);
    for (std::map<int, float>::iterator itr = a.edges.begin();
         itr != a.edges.end(); ++itr) {
      //ROS_INFO("Edges for %d: %d", a.id, (*itr).first);
    }
  }
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
Graph::reconstruct_path(std::map<int, int> camefrom, int current) {
  std::vector<Eigen::Vector2d> total_path;
  total_path.push_back(vertices[current].position);
  while (camefrom[current]) {
    current = camefrom[current];
    total_path.push_back(vertices[current].position);
  }
  return total_path;
}

std::vector<Eigen::Vector2d> Graph::getPath(Node a, Node b) {
  ROS_INFO("RUN A*");
  std::vector<int> closedSet;
  std::vector<int> openSet;
  std::map<int, float> fscore;
  std::map<int, float> gscore;
  std::map<int, int> comefrom;
  std::stringstream debug_info;
  for (std::map<int, Node>::iterator itr = vertices.begin();
       itr != vertices.end(); itr++) {
    debug_info << "Node: " << (*itr).first
               << " Position :" << (*itr).second.position << std::endl;
    // ROS_INFO("%s", debug_info.str().c_str());
    debug_info.str("");
  }
  ROS_INFO("%s", debug_info.str().c_str());
  ROS_INFO("a %d x: %d y %d edges %d", a.id, (int)a.position(0),
  (int)a.position(1), vertices[a.id].edges.size());
  ROS_INFO("b %d x: %d y %d edges %d", b.id, (int)b.position(0),
  (int)b.position(1), vertices[b.id].edges.size());
  for (std::map<int, float>::iterator itr = vertices[a.id].edges.begin();
       itr != vertices[a.id].edges.end(); ++itr) {
    ROS_INFO("Edges for %d: %d", a.id, (*itr).first);
  }

  shortest_path.clear();
  openSet.push_back(a.id);
  gscore[a.id] = 0;
  fscore[a.id] = (a.position - b.position).norm();

  while (!openSet.empty()) {
    std::sort(openSet.begin(), openSet.end(),
              [&fscore](int a, int b) { return fscore[a] < fscore[b]; });
    int current = openSet.back();
    openSet.pop_back();
    if (current == b.id) {
      return reconstruct_path(comefrom, current);
    }
    ROS_INFO("Current ID: %d", current);
    // ROS_INFO("OpenSet size %d", openSet.size());
    closedSet.push_back(current);
    Node currentNode = vertices[current];
    for (std::map<int, float>::iterator itr = currentNode.edges.begin();
         itr != currentNode.edges.end(); itr++) {
      int neighbour = (*itr).first;
      ROS_INFO("Check neighbour %d", neighbour);
      if (std::find(closedSet.begin(), closedSet.end(), neighbour) ==
          closedSet.end()) {
        float temp_gscore = gscore[current] + currentNode.edges[current];
        if ( std::find(openSet.begin(), openSet.end(), neighbour) == openSet.end() ) {
          ROS_INFO("Add to openset: %d", neighbour);
          openSet.push_back(neighbour);
        } else if ((temp_gscore >= gscore[neighbour])) {
          comefrom[neighbour] = current;
          gscore[neighbour] = temp_gscore;
          fscore[neighbour] = gscore[neighbour];
        }
      }
    }
  }
  return std::vector<Eigen::Vector2d>();
}

Eigen::Vector2d Graph::getNode(int id) {
  return (*vertices.find(id)).second.position;
}

std::map<int, Node>::iterator Graph::begin() { return vertices.begin(); }

std::map<int, Node>::iterator Graph::end() { return vertices.end(); }
