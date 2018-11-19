/*
Mapping
*/

// All imports and whatnot
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>

#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <cmath>

ros::Publisher pose_publisher;
ros::Publisher velocity_publisher;
ros::Publisher marker_pub;
ros::Publisher map_pub;
ros::Subscriber pose_sub;
ros::Subscriber map_sub;
ros::Subscriber laser_sub;

bool started = false;
double ips_x; //pos
double ips_y;
double ips_yaw; //facing direction

short sgn(int x) { return x >= 0 ? 1 : -1; }

// WHAT OTHER STUFF HERE?
const int pix_to_m = 50; // X pixels on the map grid = 1m
const int map_width = 20; //meters total width
const int map_height = 20;
const int laser_throttle = 2; //every X msgs, use 1
double sensitivity = pix_to_m/400;
double yaw_tolerance = 0.1; //anything more than 0.5 rotations / frame = bad data. disgard.

int center_x = pix_to_m*map_width/2;
int center_y = pix_to_m*map_height/2;
int row_length = map_width * pix_to_m;

nav_msgs::MapMetaData info;
//map centered around 0,0
int8_t map[map_width*pix_to_m*map_height*pix_to_m] = {-1}; //initialize all to be -1

////////////// CURRENT LASER SCAN VARIABLE:??
sensor_msgs::LaserScan laser_scan;
double angle_min, angle_max, angle_increment;
double range_min, range_max; 

//........Useful Math Tools Stuff

//converts meter distance to grid position/distance
int m_to_grid(double ips)
{
	return int(round(ips*pix_to_m));
}

// rotate a vector [dist] length about point [0,0] of angle [ang_rad]
void rotate_point(double ang_rad, double dist, int &x1, int &y1) 
{
	double s = sin(ang_rad);
	double c = cos(ang_rad);

	x1 = int(round(c*dist)); //round to nearest int
	y1 = int(round(s*dist));
}

void mark_point(int8_t* grid_point, bool clear)
{

	if (*grid_point == -1) //if a point is marked, at least it should be 100
	{
		*grid_point = 90;
	}

	if (*grid_point >= 0)
	{
		if (clear) //subtract by an amount, min 1
		{
			int num1 = int(*grid_point * sensitivity +0.5);
			int num2 = 1;
			*grid_point -= (num1<num2)?num2:num1;
		}
		if (!clear)
		{
			*grid_point =100;
		}
		
		if (*grid_point < 0) *grid_point = 0;
		if (*grid_point > 100) *grid_point = 100;
	}

}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) 
{

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


//Callback function for laser scan
int laser_counter = laser_throttle; //start at max to get a scan right now.
int num_of_scans = 0;
void laser_callback(const sensor_msgs::LaserScan &msg) 
{

	if (!started) return;
	
	if (laser_counter == laser_throttle) 
	{
		laser_scan = msg;

		angle_min = laser_scan.angle_min;
		angle_max = laser_scan.angle_max;
		angle_increment = laser_scan.angle_increment;
		
		//limits on the ranges we want to take
		range_min = 0.5;
		range_max = 3;
		
		double angle = ips_yaw + angle_min; //starting angle - facing direction of robot + left reach of scan area
		int points_added_to_map = 0;
		int rng_counter = 0;

		for (std::vector<float>::iterator it = laser_scan.ranges.begin();
				 it != laser_scan.ranges.end(); ++it)
		{
			double dist = *it;

			bool max_range = false;
			if (dist != dist)
			{
				dist = 4; //check for nan - set as full dist
				max_range = true;
				continue; //skip nan's
			}
			if (dist >= range_min && dist <= range_max)
			{
				dist = m_to_grid(dist);
				
				//find "endpoint" of the current scan.
				//current position, rotate by (yaw+scan_angle = angle)
				int start_x = m_to_grid(ips_x);
				int start_y = m_to_grid(ips_y);
				int end_x, end_y;
				rotate_point(angle, dist, end_x, end_y);
				end_x += start_x;
				end_y += start_y;

				//vectors of the points where bresenham says its clear
				std::vector<int> x_clear;
				std::vector<int> y_clear;
				bresenham(start_x, start_y, end_x, end_y, x_clear, y_clear);

				//add bresenham result to map:
				int grid_x, grid_y;
				std::vector<int>::iterator i,j;
				for (i=x_clear.begin(), j=y_clear.begin(); 
					i!=x_clear.end() && j!=y_clear.end(); ++i, ++j) 
				{
					//center around [pix_to_m*(map_width/2), pix_to_m*(map_height/2)]
					grid_x = center_x+*i;
					grid_y = center_y+*j;

					//check out of bounds
					if ( grid_x >= map_width*pix_to_m || grid_x < 0 || grid_x >= map_height*pix_to_m || grid_y < 0)
					{

						continue;
					}

					//check if last point
					
					if (std::next(i) == x_clear.end() && !max_range)
					{
						//last point
						//ROS_INFO("last point x,y: (%i, %i)", grid_x, grid_y);
						mark_point(&map[(grid_y)*row_length + (grid_x)], false);
						//map[(grid_y)*row_length + (grid_x)] = 100;
					}
					else 
					{
						points_added_to_map++;
						mark_point(&map[(grid_y)*row_length + (grid_x)], true);
						//map[(grid_y)*row_length + (grid_x)] = 0;
					}

				}


				rng_counter++;

			}

			angle += angle_increment;
		}	
		//ROS_INFO("got scan");
		nav_msgs::OccupancyGrid newGrid;
		newGrid.info = info;
		std::vector<int8_t> a(map, map+(map_width*pix_to_m*map_height*pix_to_m));
		newGrid.data = a;

		map_pub.publish(newGrid);


		num_of_scans++;
		laser_counter = 0;
	}
	else { laser_counter++; }
}

//Callback function for the Position topic (SIMULATION)
/*
void pose_callback(const gazebo_msgs::ModelStates &msg) {

  started = true;
  int i;
  for (i = 0; i < msg.name.size(); i++)
    if (msg.name[i] == "mobile_base")
      break;

  ips_x = msg.pose[i].position.x;
  ips_y = msg.pose[i].position.y;
  ips_yaw = tf::getYaw(msg.pose[i].orientation);
  //ROS_INFO("got msg");

}
*/
//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	started = true;

	geometry_msgs::Quaternion quat = msg.pose.pose.orientation;
	if (abs(quat.x)>1 || abs(quat.y)>1 || abs(quat.z)>1 || abs(quat.w)>1)
	{
		//bad msg
		started = false;
		ROS_INFO("bad msg");
		return;
	}
	ips_x = msg.pose.pose.position.x; // Robot X psotition
	ips_y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
}



int main(int argc, char **argv) {

  // Initialize the ROS framework
  ros::init(argc, argv, "main_control");
  ros::NodeHandle n;

  //Subscribe to the desired topics and assign callbacks

  //REAL      @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

  //SIMULATION      @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
  

  laser_sub = n.subscribe("/scan", 1, laser_callback);
  //map_sub = n.subscribe("/map", 1, map_callback);

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_map",1);

  // map metadata info
	info.resolution = 1.0/double(pix_to_m);
	info.width = pix_to_m*map_width;
	info.height = pix_to_m*map_height;

	geometry_msgs::Pose my_pose;
	geometry_msgs::Point point;
	point.x = 0-map_width/2;
	point.y = 0-map_height/2;
	point.z = 0;
	my_pose.position = point;

	//for some reason cannot initalize map ={-1}, so do it manually as a workaround for now
	for (int i=0; i<map_width*pix_to_m*map_height*pix_to_m; i++)
	{
		map[i] = -1;
	}

	info.origin = my_pose;
  //pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
  //marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  //Velocity control variable
  geometry_msgs::Twist vel;

  //Set the loop rate
  ros::Rate loop_rate(20); //20Hz update rate

  ROS_INFO("Program Started");
  //////////test publisher node
  ros::Publisher test_pub = n.advertise<std_msgs::String>("aa",1);

	while (ros::ok()) {
    loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce();   // Check for new messages
  }

  return 0;
}
