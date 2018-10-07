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

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>


//Callback function for the Position topic 
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
	//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}
double X, Y, Yaw;
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

  X = msg->pose.pose.position.x; // Robot X psotition
	Y = msg->pose.pose.position.y; // Robot Y psotition
 	Yaw = tf:: getYaw(msg->pose.pose.orientation); // Robot Yaw
}



int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate


    ros::spinOnce();
    int dist = 5;
    double travel_x = 0;
    double lastX = X;
    double rotate_z = 0;
    double lastYaw = Yaw;
    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    
    	//Main loop code goes here:
      if (travel_x == 0)
        {
          vel.linear.x = 0.2;
          travel_x += X-lastX;
        }
      if (travel_x > dist)
        {
          travel_x = -1;
          vel.linear.x = 0;
          rotate_z = 0;
        }
      if (rotate_z == 0 && travel_x == -1)
        {
          vel.angular.z = 0.2;
          rotate_z += Yaw-lastYaw;
        }
      if (rotate_z > 3.14159265/2.0)
        {
          travel_x = 0;
          vel.angular.z = 0;
        }

      // vel.linear.x = 0.2;
    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
