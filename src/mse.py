#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rospy import Time
import matplotlib.pyplot as plt
import math
import numpy as np

mse = []
time = []
def ips_callback(data):
    global ips_x
    global ips_y
    ips_x = data.pose.position.x
    ips_y = data.pose.position.y
    print( "ips: " + repr(ips_x) + ", " + repr(ips_y))
    print( "pf: " + repr(pf_x) + ", " + repr(pf_y))
    print(calculateMSE())

def pose_callback(data):
    global pf_x
    global pf_y
    pf_x = data.pose.position.x
    pf_y = data.pose.position.y
#    print( "pf: " + repr(pf_x) + ", " + repr(pf_y))
#    calculateMSE()
#    print(calculateMSE())

def calculateMSE():
    global mse
    global now
    mse.append(math.sqrt( math.pow((ips_x - pf_x), 2)  + math.pow((ips_y - pf_y), 2) ) )
    time.append(rospy.get_time())

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('mean_squared_error', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("ips_pose", PoseStamped, ips_callback)
    rospy.Subscriber("pose", PoseStamped, pose_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    plt.ylabel("Mean squared error")
    plt.xlabel("Time")
    plt.plot(time, mse)
    # plt.plot(time, np.cumsum(mse))
    plt.show()
