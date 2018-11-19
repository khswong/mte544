#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
import math

ips_x = 0;
ips_y = 0;
ips_t = 0;

pf_x = 0;
pf_y = 0;
pf_t = 0;

def ips_callback(data):
    ips_x = data.pose.position.x
    ips_y = data.pose.position.y

def pose_callback(data):
    pf_x = data.pose.position.x
    pf_y = data.pose.position.y

def calculateMSE():
    return sqrt( math.pow((ips_x - pf_x), 2)  + math.pow((ips_y - pf_y), 2) )

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('mean_squared_error', anonymous=True)

    rospy.Subscriber("ips_pose", PoseStamped, ips_callback)
    rospy.Subscriber("pose", PoseStamped, pose_callback)
    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()

if __name__ == '__main__':
    listener()
