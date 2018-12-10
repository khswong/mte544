// Simple undirected graph class

#include "graph.h"
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

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


// A* graph search
bool Graph::isReachable(Node a, Node b) {
  ROS_INFO("RUN A*");
  std::vector<int> closedSet;
  std::vector<int> openSet;
  std::vector<float> fscore;
  std::vector<float> gscore;
  std::vector<int> comefrom;
  shortest_path.clear();
  openSet.push_back(a.id);
  gscore.push_back(0.0);
  fscore.push_back((a.position - b.position).norm());
  int current = a.id;

  while (!openSet.empty()) {
    ROS_INFO("openset.length: %d", openSet.size());

    ROS_INFO("current: %d", current);
    ROS_INFO("b id: %d", b.id);
    ROS_INFO("NOT EXIT CONDITION YET");
    ros::Duration(0.1).sleep();

    current = *std::min_element(openSet.begin(), openSet.end());
    if (current == b.id)
    {
      break;
    }
    openSet.erase( std::find(openSet.begin(), openSet.end(), current) );
    closedSet.push_back(current);

    ROS_INFO("current node %d", current);
    Node currentNode = this->vertices.find(current)->second;
    // For neighbours beside currentnode

    for (std::map<int, float>::iterator itr = currentNode.edges.begin();
         itr != currentNode.edges.end(); ++itr) {
    
      const int neighbour = (*itr).first;
      ROS_INFO("Check neighbour %d", neighbour);
    
      // And of those neighbours is not within the closed set
      if (std::find(closedSet.begin(), closedSet.end(), neighbour) !=
          closedSet.end()) {
        if (std::find(closedSet.begin(), closedSet.end(), neighbour) !=
            closedSet.end()) {
          
          openSet.push_back(neighbour);
        } else if ( gscore[current] + ((*currentNode.edges.find(neighbour)).second) >
                   gscore[neighbour]) {
          ROS_INFO("Found some thing");

          // Insert the distance from the beginning
          gscore.insert(gscore.begin(), neighbour,
                        gscore[current] + (*currentNode.edges.find(neighbour)).second);

          // Insert the distance to the end (heuristic)
          fscore.insert(
              fscore.begin(), neighbour,
              gscore[neighbour] +
                  ((*vertices.find(neighbour)).second.position - b.position)
                      .norm());

          comefrom[neighbour] = current;
        }
      }
    }
  }

  ROS_INFO("EXITED\n");
  if (current == b.id) {
    shortest_path.push_back(current);
    while (current != a.id)
      {
        current = comefrom[current];
        shortest_path.push_back(current);
      }
    return true;
  } else {
    return false;
  }
}

void Graph::deleteVertex(Node a) {
  for (std::map<int, Node>::iterator itr = vertices.begin();
       itr != vertices.end(); ++itr) {
    deleteEdge((*itr).second, a);
  }
  vertices.erase(a.id);
}

void Graph::deleteEdge(Node a, Node b) {
  a.edges.erase(a.edges.find(b.id));
  b.edges.erase(a.edges.find(b.id));
}

std::vector<int> Graph::getPath(){ return shortest_path; }
Eigen::Vector2d Graph::getNode(int id){return (*vertices.find(id)).second.position;}

std::map<int, Node>::iterator Graph::begin() { return vertices.begin(); }

std::map<int, Node>::iterator Graph::end() { return vertices.end(); }
