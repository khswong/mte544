//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos

/*
catkin_make
rosrun map_server map_server map_sim.yaml
rosrun turtlebot_example planning
roslaunch turtlebot_example turtlebot_gazebo.launch
*/
//
// //////////////////////////////////////////////////////////

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include "prm.h"
#include <algorithm>
#include <eigen3/Eigen/Dense>
using std::showpoint;
using std::fixed;

ros::Publisher marker_pub;
static PrmPlanner prm_planner;

#define TAGID 0

double X, Y, Yaw;
bool map_is_setup = false;
std::vector<Eigen::Vector2d> waypoints;

// Callback function for the Position topic (LIVE)

void pose_callback(const gazebo_msgs::ModelStates &msg) {
  // This function is called when a new position message is received
  int i;
  for (i = 0; i < msg.name.size(); i++)
    if (msg.name[i] == "mobile_base")
      break;

  X = msg.pose[i].position.x;                // Robot X psotition
  Y = msg.pose[i].position.y;                // Robot Y psotition
  Yaw = tf::getYaw(msg.pose[i].orientation); // Robot Yaw

  // std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl;
}

// Example of drawing a curve
void drawCurve(int k) {
  // Curves are drawn as a series of stright lines
  // Simply sample your curves into a series of points

  double x = 0;
  double y = 0;
  double steps = 50;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  // each curve must have a unique id or you will overwrite an old ones
  lines.id = k;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.1;
  lines.color.r = 1.0;
  lines.color.b = 0.2 * k;
  lines.color.a = 1.0;

  // generate curve points
  for (int i = 0; i < steps; i++) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0; // not used
    lines.points.push_back(p);

    // curve model
    x = x + 0.1;
    y = sin(0.1 * i * k);
  }

  // publish new curve
  marker_pub.publish(lines);
}

inline void setupPRM() {
#if 1
  Eigen::Vector2d offset;
  offset << 1, 5;
  // Setup PRM
  std::vector<Eigen::Vector2d> goals;
  goals.push_back(Eigen::Vector2d(X, Y) + offset);
  goals.push_back(Eigen::Vector2d(4, 0) + offset);
  goals.push_back(Eigen::Vector2d(8, 0) + offset);
  goals.push_back(Eigen::Vector2d(8, -4) + offset);

#else

  double pts[] = {
      0, 0,                                    // start point
      0, -4, 6, -4, 6, -2, 5, -1, 4, -1, 4, 0, // first waypoint
      4, -1, 5, -1, 6, -3, 8, -4,              // second waypoint
      6, -3, 6, -1, 8, 0                       // final waypoint

  };
  // Setup PRM
  for (int i = 0; i < sizeof(pts) / sizeof(pts[i]); i += 2) {
    waypoints.push_back(Eigen::Vector2d(pts[i], pts[i + 1]));
  }

  ROS_INFO("frick 0");
#endif
  std::vector<Eigen::Vector2d> campaign;
  for (std::vector<Eigen::Vector2d>::iterator itr = goals.begin();
       itr != goals.end() - 1; itr++) {
    prm_planner.setPos(*itr);
    prm_planner.setGoal(*(std::next(itr, 1))); // breaks here
    // prm_planner.sampleMilestones();
    ROS_INFO("frick 3");
    campaign = prm_planner.getPath();
    ROS_INFO("frick 4");
    waypoints.insert(waypoints.end(), campaign.begin(), campaign.end());
    ROS_INFO("frick 5");
  }
  // waypoints = goals;
  map_is_setup = true;
  ROS_INFO("SET UP MAP.");
}

// Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid &msg) {
  // This function is called when a new map is received

  // you probably want to save the map into a form which is easy to work with
  ROS_INFO("GOT MAP:..");
  prm_planner.setMap(msg.data, (int)msg.info.width, (int)msg.info.height);
  prm_planner.setRes(msg.info.resolution);
  setupPRM();
}

int main(int argc, char **argv) {
  // Eigen::initParallel();
  // Initialize the ROS framework
  ros::init(argc, argv, "main_control");
  ros::NodeHandle n;

  // Subscribe to the desired topics and assign callbacks
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  ros::Subscriber pose_sub =
      n.subscribe("/gazebo/model_states", 1, pose_callback);

  // Setup topics to Publish from this node
  ros::Publisher velocity_publisher =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  // Velocity control variable
  geometry_msgs::Twist vel;

  /*
  General Algorithm

  Get occupancy grid
  Do PRM
    select random nodes, filter, etc
    find edges, collision, etc
    get shortest path

  //loop controller
  check distance to next node:
    while not there:
      rotate to face the next node
      move towards next node at current theta

  */

  double rad2deg = 360 / (2 * M_PI);
  double deg2rad = 2 * M_PI / 360;

  double tolerance = 0.25;
  double angle_tolerance = 15 * deg2rad;
  double angle_tolerance2 = 10 * deg2rad;
  double base_angle_speed = 0.1;
  double speed_gain = 0.05;
  double base_speed = 0.5;
  double angle_gain = 0.5;

  double dx, dy, dist_to_pt, angle_to_pt, angle_diff;

  bool need_to_rotate = false;
  bool done_rotation = false;
  // Set the loop rate
  ros::Rate loop_rate(20); // 20Hz update rate
  ROS_INFO("~~START~~ ^-^");

  while (ros::ok()) {
    loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce();   // Check for new messages
    std::cout << "\n\n WAYPTS: " << waypoints.size();

    // while (!map_is_setup); // Wait until a map is set up
    while (waypoints.empty())
      ; // Just stop if no waypoints

    // current goal
    Eigen::Vector2d currentGoal = *waypoints.begin();
    dx = currentGoal(0) - X;
    dy = currentGoal(1) - Y;
    dist_to_pt = sqrt(pow(dx, 2) + pow(dy, 2));

    if (std::abs(dist_to_pt) > tolerance) {
      // Move towards next goal
      angle_to_pt = atan2(dy, dx);
      angle_diff = std::fmod((angle_to_pt - Yaw + 3 * M_PI), (2 * M_PI)) - M_PI;

      if (std::abs(angle_diff) > angle_tolerance) {
        need_to_rotate = true;
      }
      if (need_to_rotate && done_rotation == false) {
        // Rotate first
        if (std::abs(angle_diff) < angle_tolerance2) {
          done_rotation = true;
          need_to_rotate = false;
        }
        std::cout << "\n ROTATING";
        std::cout << "\n  angle_to_pt: " << angle_to_pt * rad2deg
                  << "\n  angle diff: " << angle_diff * rad2deg << "\n  pt-yaw "
                  << (angle_to_pt - Yaw) * rad2deg << "\n   +pi "
                  << (angle_to_pt - Yaw + M_PI) * rad2deg << "\n  fmod:"
                  << std::fmod((angle_to_pt - Yaw + M_PI), (2 * M_PI)) *
                         rad2deg;
        int sign = angle_diff / std::abs(angle_diff);
        vel.angular.z = base_angle_speed * sign + angle_diff * angle_gain;
        vel.linear.x = 0;
      } else {
        done_rotation = false;
        ///////////// Move towards point
        std::cout << "\n MOVING STRAIGHT";
        vel.angular.z = 0;
        vel.linear.x = base_speed + speed_gain * dist_to_pt;
      }

    } else {
      // Reached the goal, move onto next one.
      waypoints.erase(waypoints.begin());
      currentGoal = *waypoints.begin();
    }

    std::cout << showpoint << fixed;
    std::cout << "\nRobot: x:" << X << " y:" << Y << " angle:" << Yaw * rad2deg;
    std::cout << "\nTarg:  x:" << currentGoal(0) << " y:" << currentGoal(1)
              << " dist: " << dist_to_pt << " ang_dif:" << angle_diff * rad2deg;
    std::cout << "\nSpeed: x:" << vel.linear.x << " y:" << vel.linear.y
              << " angle:" << vel.angular.z;

    velocity_publisher.publish(vel); // Publish the command velocity
  }

  return 0;
}
