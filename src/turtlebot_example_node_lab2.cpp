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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Dense>

#include "particle_filter.h"

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

// Singleton I guess?

#define N_PARTICLES 10
ParticleFilter pf(N_PARTICLES);

geometry_msgs::PoseStamped pose;
double ips_x;
double ips_y;
double ips_yaw;

short sgn(int x) { return x >= 0 ? 1 : -1; }

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates &msg) {
    int i;
    Eigen::Vector3d measurement;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;

    ips_x = msg.pose[i].position.x;
    ips_y = msg.pose[i].position.y;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
    measurement << ips_x, ips_y, ips_yaw;
    pf.measurementUpdate(measurement);
    ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);
}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid &msg) {
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with
}

void velocity_callback(const geometry_msgs::Twist &msg ){
  Eigen::Vector3d input;
  Eigen::Matrix3d rot;
  double yaw = tf::getYaw(pose.pose.orientation);
  input << msg.linear.x, msg.linear.y, msg.angular.z;
  rot << cos(yaw), 0 - sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  pf.particleUpdate(rot*input);
  pf.calculateStats();
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s)
            y0 += sgn(dy2);
        else
            x0 += sgn(dx2);
        if (d < 0)
            d += inc1;
        else {
            d += inc2;
            if (s)
                x0 += sgn(dx2);
            else
                y0 += sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc, argv, "main_control");
    ros::NodeHandle n;

    ROS_INFO("Subscribe");
    //Subscribe to the desired topics and assign callbacks
#ifndef LIVE
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
#else
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/odom", 1, map_callback);
#endif
    ros::Subscriber velocity_sub = n.subscribe("/cmd_vel_mux/input/teleop", 1, velocity_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
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
    //only if using a MESH_RESOURCE marker type:

    //Velocity control variable
    geometry_msgs::Twist vel;
    //Set the loop rate
    ros::Rate loop_rate(20); //20Hz update rate
    Eigen::Vector3d r;
    r << 0.3, 0.3, 0.3;
    pf.setR(r);

    while (ros::ok()) {
      loop_rate.sleep(); //Maintain the loop rate
      ros::spinOnce();   //Check for new messages
      //Main loop code goes here:
      //Prediction update
      //pf.particleUpdate(input);
      Eigen::Vector3d mean = pf.getMean();
      pose.pose.position.x = mean(0);
      pose.pose.position.y = mean(1);
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(mean(2));
      pose_publisher.publish(pose);
      std::vector<Eigen::Vector3d> particles = pf.getParticles();
      std::vector<Eigen::Vector3d>::iterator particle_iter;
      for (particle_iter = particles.begin(); particle_iter < particles.end();
           particle_iter++) {
        geometry_msgs::Point point;
        point.x = (*particle_iter)(0);
        point.y = (*particle_iter)(1);
        point.z = 0;
        marker.points.push_back(point);
        }
      marker_pub.publish(marker);
      //velocity_publisher.publish(vel); // Publish the command velocity
    }
    return 0;
}


