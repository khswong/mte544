/*
Mapping Pseudocode
Nov 1 2018

todo:
implement laser scan callback: takes the scan and puts it in something to use
main loop:
	take current laser scan and update current map using bresenhem

*/

// All imports and whatnot
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>

#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <math.h>

ros::Publisher pose_publisher;
ros::Publisher velocity_publisher;
ros::Publisher marker_pub;
ros::Publisher map_pub;
ros::Subscriber pose_sub;
ros::Subscriber map_sub;
ros::Subscriber laser_sub;

double ips_x; //pos
double ips_y;
double ips_yaw; //facing direction

short sgn(int x) { return x >= 0 ? 1 : -1; }

// WHAT OTHER STUFF HERE?
const int pix_to_m = 50; // X pixels on the map grid = 1m
const int map_width = 10; //meters total width
const int map_height = 10;
const int laser_throttle = 1; //every X msgs, use 1
float sensitivity = pix_to_m/200;

int center_x = pix_to_m*map_width/2;
int center_y = pix_to_m*map_height/2;
int row_length = map_width * pix_to_m;

nav_msgs::MapMetaData info;
//map centered around 0,0
int8_t map[map_width*pix_to_m*map_height*pix_to_m] = {-1}; //initialize all to be -1

////////////// CURRENT LASER SCAN VARIABLE:??
sensor_msgs::LaserScan laser_scan;
float angle_min, angle_max, angle_increment;
float range_min, range_max; 

//........Useful Math Tools Stuff

//converts meter distance to grid position/distance
int m_to_grid(double ips)
{
	return int(round(ips*pix_to_m));
}

// rotate a vector [dist] length about point [0,0] of angle [ang_rad]
void rotate_point(float ang_rad, float dist, int &x1, int &y1) 
{
	float s = sin(ang_rad);
	float c = cos(ang_rad);

	x1 = int(round(c*dist)); //round to nearest int
	y1 = int(round(s*dist));
}

void mark_point(int8_t* grid_point, bool clear)
{

	if (*grid_point == -1) //if a point is marked, at least it should be 100
	{
		*grid_point = 100;
	}

	if (*grid_point > 0)
	{
		if (clear) //subtract by an amount, min 1
		{
			int num1 = int(*grid_point * sensitivity +0.5);
			int num2 = 1;
			*grid_point -= (num1<num2)?num2:num1;
		}
		if (!clear)
		{
			*grid_point = 100;
		}
		
		if (*grid_point < 0) *grid_point = 0;
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
	if (laser_counter == laser_throttle) 
	{
		//TODO: do stuff here
		laser_scan = msg;

		angle_min = laser_scan.angle_min;
		angle_max = laser_scan.angle_max;
		angle_increment = laser_scan.angle_increment;
		
		//make sure all scans are good, don't take the nan's
		range_min = laser_scan.range_min;
		range_max = laser_scan.range_max;
		
		
		float angle = ips_yaw + angle_min; //starting angle - facing direction of robot + left reach of scan area
		int points_added_to_map = 0;
		int rng_counter = 0;

		for (std::vector<float>::iterator it = laser_scan.ranges.begin();
				 it != laser_scan.ranges.end(); ++it)
		{
			float dist = *it;
			//ROS_INFO("dist: %f", dist);

			bool max_range = false;
			if (dist != dist)
			{
				dist = 4; //check for nan - set as full dist
				max_range = true;
			}
			if (dist >= range_min && dist <= range_max)
			{
				//dist = m_to_grid(range_max);
				dist = m_to_grid(dist);
				//ROS_INFO("dist: %i\n", int(dist));
				
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
				//int temp_count = 0;
				int grid_x, grid_y;
				std::vector<int>::iterator i,j;
				for (i=x_clear.begin(), j=y_clear.begin(); 
					i!=x_clear.end() && j!=y_clear.end(); ++i, ++j) 
				{
					//center around [pix_to_m*(map_width/2), pix_to_m*(map_height/2)]
					grid_x = center_x+*i;
					grid_y = center_y+*j;

					//ROS_INFO("point x,y: (%i, %i)", grid_x, grid_y);

					//check out of bounds
					if ( grid_x >= map_width*pix_to_m || grid_x < 0 || grid_x >= map_height*pix_to_m || grid_y < 0)
					{
						//ROS_INFO("center: x: #%i\n", center_x);
						//ROS_INFO("x: #%i\n", center_x+*i);
						//ROS_INFO("y: #%i\n", center_y+*j);
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
				//if not nan sensor reading,last point (ie the wall), mark as occupied
				//last point
				int last_x = center_x+end_x;
				int last_y = center_y+end_y;
				//mark_point(&map[(last_y)*row_length + (last_x)], -100);

				rng_counter++;
				//////////////////
			}

			angle += angle_increment;
		}	

		nav_msgs::OccupancyGrid newGrid;
		newGrid.info = info;
		std::vector<int8_t> a(map, map+(map_width*pix_to_m*map_height*pix_to_m));
		newGrid.data = a;

		map_pub.publish(newGrid);


		num_of_scans++;
		ROS_INFO("got scan #%i\n", num_of_scans);
		//ROS_INFO("num of points in range: %i\n", rng_counter);
		//for(int i=0; i<500; i++)
			//ROS_INFO("map: pt:%i, val:%i", i, map[i]);
		//ROS_INFO("pts added to map: %i\n\n", points_added_to_map);
		laser_counter = 0;
	}
	else { laser_counter++; }
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates &msg) {

  int i;
  for (i = 0; i < msg.name.size(); i++)
    if (msg.name[i] == "mobile_base")
      break;

  ips_x = msg.pose[i].position.x;
  ips_y = msg.pose[i].position.y;
  ips_yaw = tf::getYaw(msg.pose[i].orientation);

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



int main(int argc, char **argv) {

  // Initialize the ROS framework
  ros::init(argc, argv, "main_control");
  ros::NodeHandle n;

  //Subscribe to the desired topics and assign callbacks
  pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
  laser_sub = n.subscribe("/scan", 1, laser_callback);
  //map_sub = n.subscribe("/map", 1, map_callback);

  //Setup topics to Publish from this node
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_map",1);

  // map metadata info
	info.resolution = 1.0/float(pix_to_m);
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


  //////////test publisher node
  ros::Publisher test_pub = n.advertise<std_msgs::String>("aa",1);

	while (ros::ok()) {
    loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce();   // Check for new messages


    //move around in a circle
    vel.linear.x = 0.05;  // set linear speed
    vel.angular.z = -0.05; // set angular speed
	//velocity_publisher.publish(vel); // Publish the command velocity

    ///////////////// waste man garbo
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hi";
    msg.data = ss.str();
    test_pub.publish(msg);
  }

  return 0;
}

/*
while (1)
{
	curr_pos = new Vector(sub_IPS_pose); //might need to map to 500x500 or something...
	facing_direction = vector_to_angle(sub_bot_direction); //might start as a vector, convert to angle wrt "x axis"
	laser_scan = new LaserScan(sub_bot_direction);
	


	angle = angle_min;
	for (int i=0; i<ranges.length(); i++)
	{
		range = ranges[i];
		if (range >= range_min && range <= range_max)
		{
			angle_ref = facing_direction - angle;
			start_point = curr_pos;
			end_point = new Vector(start_point.x * cos(angle_ref), start_point.y * sin(angle_ref));

		}
		angle += angle_increment;
	}	
}
*/

