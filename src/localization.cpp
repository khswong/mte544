//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <ros/time.h>

#include <eigen3/Eigen/Dense>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include "particle_filter.h"

ros::Publisher pose_publisher;
ros::Publisher marker_pub;
ros::Publisher truepose_publisher;

// Singleton I guess?

#define N_PARTICLES 150
ParticleFilter pf(N_PARTICLES);

geometry_msgs::PoseStamped pfpose;
geometry_msgs::PoseStamped ipspose;
visualization_msgs::Marker marker;
double ips_x;
double ips_y;
double ips_yaw;

Eigen::Vector3d measurement = Eigen::Vector3d::Zero();

short sgn(int x) { return x >= 0 ? 1 : -1; }

#define LIVE 1
// Callback function for the Position topic (SIMULATION)
#ifndef LIVE
void pose_callback(const gazebo_msgs::ModelStates &msg) {
  int i;
  for (i = 0; i < msg.name.size(); i++)
    if (msg.name[i] == "mobile_base")
      break;

  ips_x = msg.pose[i].position.x;
  ips_y = msg.pose[i].position.y;
  ips_yaw = tf::getYaw(msg.pose[i].orientation);
  measurement << ips_x, ips_y, ips_yaw;
  pf.measurementUpdate(measurement);
  // Also publish the true state
  ipspose.header.frame_id = "map";
  ipspose.pose = msg.pose[i];
  ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);
}
#else
// Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  ips_x = msg.pose.pose.position.x;              // Robot X psotition
  ips_y = msg.pose.pose.position.y;              // Robot Y psotition
  ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

  measurement << ips_x, ips_y, ips_yaw;
  pf.measurementUpdate(measurement);
  // Also publish the true state
  ipspose.header.frame_id = "map";
  ipspose.pose = msg.pose.pose;
  ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);
}
#endif

void velocity_callback(const geometry_msgs::Twist &msg) {
  static ros::Time time_now = ros::Time::now();
  static ros::Time time_prev;
  Eigen::Vector3d input;
  Eigen::Matrix3d rot;
  double yaw = tf::getYaw(pfpose.pose.orientation);
  time_prev = time_now;
  time_now = ros::Time::now();
  ros::Duration dt = time_now - time_prev;
  input << msg.linear.x, msg.linear.y, msg.angular.z;

  if (!input.isZero()) {
    rot << cos(yaw), 0 - sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    pf.particleUpdate(rot * input * dt.toSec());
    pf.calculateStats();
  }
}

static void marker_setup() {
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  return;
}

int main(int argc, char **argv) {
  // Initialize the ROS framework
  ros::init(argc, argv, "main_control");
  ros::NodeHandle n;

  ROS_INFO("Subscribe");
// Subscribe to the desired topics and assign callbacks
#ifndef LIVE
  ros::Subscriber pose_sub =
      n.subscribe("/gazebo/model_states", 1, pose_callback);
#else
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
#endif
  ros::Subscriber velocity_sub =
      n.subscribe("/cmd_vel_mux/input/teleop", 1, velocity_callback);

  // Setup topics to Publish from this node
  ros::Publisher velocity_publisher =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
  truepose_publisher =
      n.advertise<geometry_msgs::PoseStamped>("/ips_pose", 1, true);
  marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  marker_setup();

  // Velocity control variable
  geometry_msgs::Twist vel;
  // Set the loop rate
  ros::Rate loop_rate(20); // 20Hz update rate
  Eigen::Vector3d r;
  r << 0.1, 0.1, 0.01;
  pf.setR(r);

  while (ros::ok()) {
    loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce();   // Check for new messages
    // Main loop code goes here:

    // Return the mean position and publish to pose
    Eigen::Vector3d mean = pf.getMean();
    Eigen::Vector3d median = pf.getMedian();
    pfpose.header.frame_id = "map";
    pfpose.pose.position.x = mean(0);
    pfpose.pose.position.y = mean(1);
    pfpose.pose.orientation = tf::createQuaternionMsgFromYaw(mean(2));
    pose_publisher.publish(pfpose);
    truepose_publisher.publish(ipspose);
    // Visualize the particle filter
    std::vector<Eigen::Vector3d> particles = pf.getParticles();
    std::vector<Eigen::Vector3d>::iterator particle_iter;
    marker.points.clear();
    for (particle_iter = particles.begin(); particle_iter < particles.end();
         particle_iter++) {
      geometry_msgs::Point point;
      point.x = (*particle_iter)(0);
      point.y = (*particle_iter)(1);
      point.z = 0;
      marker.points.push_back(point);
    }
    marker_pub.publish(marker);
    // velocity_publisher.publish(vel); // Publish the command velocity
  }
  return 0;
}
