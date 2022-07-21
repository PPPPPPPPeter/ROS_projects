#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #

def callback(dt):

    forwards_threshold = 0.8 # Laser scan range threshold for forward motion 
    backwards_threshold = 0.1 # Laser scan range threshold for backwards motion 

    if dt.ranges[0]>forwards_threshold and dt.ranges[15]>forwards_threshold and dt.ranges[345]>forwards_threshold:
        move.linear.x = 0.5 # continues
    else:
        move.linear.x = 0.0 # stops
        move.linear.x = -0.1 # moves backwards
        
        if dt.ranges[0]>forwards_threshold and dt.ranges[15]< forwards_threshold:
            move.linear.x = 0.5 
            move.angular.z = -0.4 # rotates counter-clockwise
            
        if dt.ranges[0]>forwards_threshold and dt.ranges[345]< forwards_threshold: 
            move.linear.x = 0.5      
            move.angular.z = 0.4 # rotates clockwise

        if dt.ranges[180]<backwards_threshold :
            move.linear.x = 0.1

    pub.publish(move) # publish the move object

move = Twist() 
rospy.init_node('obstacle_avoidance_node') 
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  

sub = rospy.Subscriber("/scan", LaserScan, callback)  

rospy.spin() # Loops infinitely 