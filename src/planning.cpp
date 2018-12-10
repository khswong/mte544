//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
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

ros::Publisher marker_pub;
static PrmPlanner prm_planner;

#define TAGID 0

double X, Y, Yaw;
bool map_is_setup = false;
std::vector<Eigen::Vector2d> waypoints;


// Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  // This function is called when a new position message is received
  X = msg.pose.pose.position.x;                // Robot X psotition
  Y = msg.pose.pose.position.y;                // Robot Y psotition
  Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

  std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl;
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

inline void setupPRM()
{
  Eigen::Vector2d offset;
  offset << 1, 5;
  // Setup PRM
  std::vector<Eigen::Vector2d> goals;
  goals.push_back(Eigen::Vector2d(X,Y));

  goals.push_back(Eigen::Vector2d(4, 0) + offset);
  goals.push_back(Eigen::Vector2d(8, 0) + offset);
  goals.push_back(Eigen::Vector2d(8, -4) + offset);
  
  std::vector<Eigen::Vector2d> campaign;
  for (std::vector<Eigen::Vector2d>::iterator itr = goals.begin();
       itr != goals.end(); itr++)
  {
    prm_planner.setPos(*itr);
    prm_planner.setGoal(*(itr + 1)); //breaks here
    //prm_planner.sampleMilestones();
    ROS_INFO("frick 3");
    campaign = prm_planner.getPath();
    ROS_INFO("frick 4");
    waypoints.insert(waypoints.end(), campaign.begin(), campaign.end());
    ROS_INFO("frick 5");
  }

  map_is_setup = true;
  ROS_INFO("SET UP MAP.");
}

// Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid &msg) {
  // This function is called when a new map is received

  // you probably want to save the map into a form which is easy to work with
  ROS_INFO("GOT MAP:..");
  prm_planner.setMap(msg.data,  (int)msg.info.width, (int)msg.info.height);
  prm_planner.setRes(msg.info.resolution);
  setupPRM();

}


int main(int argc, char **argv) {
  //Eigen::initParallel();
  // Initialize the ROS framework
  ros::init(argc, argv, "main_control");
  ros::NodeHandle n;

  // Subscribe to the desired topics and assign callbacks
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

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
  double angle_tolerance = 5 * deg2rad;
  double speed = 0.1;
  double angle_gain = 0.5;

  double dx, dy, dist_to_pt, angle_to_pt;

  // Set the loop rate
  ros::Rate loop_rate(20); // 20Hz update rate
  ROS_INFO("~~START~~ ^-^");

  while (ros::ok()) {
    loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce();   // Check for new messages

    while (!map_is_setup); // Wait until a map is set up
    while (waypoints.empty()); // Just stop if no waypoints

    ROS_INFO("LOOPED ONCE.");
    // current goal
    Eigen::Vector2d currentGoal = *waypoints.begin();
    dx = currentGoal(0) - X;
    dy = currentGoal(1) - Y;
    dist_to_pt = sqrt(pow(dx, 2) + pow(dy, 2));

    if (std::abs(dist_to_pt) > tolerance) {
      // Move towards next goal
      angle_to_pt = atan2(dy, dx);
      ROS_INFO("MOVING TOWARDS NEXT GOAL. dist:%f, angle:%f", dist_to_pt, angle_to_pt);
      double angle_diff =
          std::fmod((angle_to_pt - Yaw + M_PI), (2 * M_PI)) - M_PI;
      if (std::abs(angle_diff) > angle_tolerance) {
        ///////////// Rotate first
        vel.angular.z = angle_diff * angle_gain;
        vel.linear.x = 0;
        vel.linear.y = 0;
      } else {
        ///////////// Move towards point
        vel.angular.z = 0;
        vel.linear.x = speed * cos(Yaw);
        vel.linear.y = speed * sin(Yaw);
      }

    } else {
      // Reached the current goal, move onto next one.
      waypoints.pop_back();
    }

    velocity_publisher.publish(vel); // Publish the command velocity
  }

  return 0;
}
