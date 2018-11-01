//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various
// inputs and outputs.
//
// Author: James Servos
//
// //////////////////////////////////////////////////////////

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <ros/console.h>
#include <cmath>
// Callback function for the Position topic
// void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&
// msg)
//{
// This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

//#define M_PI 3.14159265
double X, Y, Yaw;
void pose_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    // This function is called when a new position message is received

  X = msg->pose.pose.position.x;                // Robot X psotition
  Y = msg->pose.pose.position.y;                // Robot Y psotition
  Yaw = tf::getYaw(msg->pose.pose.orientation) + M_PI; // Robot Yaw + offset
}

int main(int argc, char **argv) {
  // Initialize the ROS framework
  ros::init(argc, argv, "main_control");
  ros::NodeHandle n;

  // Subscribe to the desired topics and assign callbacks
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

  // Setup topics to Publish from this node
  ros::Publisher velocity_publisher =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

  // Velocity control variable
  geometry_msgs::Twist vel;

  // Set the loop rate
  ros::Rate loop_rate(20); // 20Hz update rate

  ros::spinOnce();
  const double dist = 0.75;
  double travel_dist = 0;
  double lastX = X;
  double lastY = Y;
  double rotate_z = 0;
  double lastYaw = Yaw;

  while (ros::ok()) {
    loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce();   // Check for new messages
    ROS_INFO("X: %f Y: %f yaw: %f \n", X, Y, Yaw);
    ROS_INFO("lastX: %f lastY: %f lastyaw: %f \n", lastX, lastY, lastYaw);
    ROS_INFO("distX: %f rotate_z: %f \n", travel_dist, rotate_z);
    // Main loop code goes here:

    // Update distances
    travel_dist += sqrtf ( powf((X - lastX), 2) + powf((Y - lastY), 2));

    //
    #define THRESHOLD 0.05
    if (travel_dist > dist) {
      ROS_INFO("TURNING, rotate_z: %f", rotate_z);
      vel.linear.x = 0.00;
      rotate_z += fabs( Yaw - lastYaw );
      vel.angular.z = 0.1 + 0.5*(2.0/M_PI)*fabs(rotate_z - M_PI/2.0); // Simple P control
      if (rotate_z > (M_PI / 2.0) ) {
        travel_dist = 0;
        rotate_z = 0;
        vel.angular.z = 0;
      }
    }
    else {
      vel.linear.x = 0.1 + 0.5 * fabs(travel_dist - dist);
    }

    //Update after
    lastX = X;
    lastY = Y;
    lastYaw = Yaw;
    //Publish
    velocity_publisher.publish(vel); // Publish the command velocity
  }

  return 0;
}
