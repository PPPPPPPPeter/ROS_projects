#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


node_name = "task1_sub"
topic_name = "odom"

first_time_subscriber = True
print ("------"+str(first_time_subscriber))

position_x0 = 0
position_y0 = 0
yaw0 = 0
printing_stuff = 0


def callback_function(sub_msg):
    
    position_x = sub_msg.pose.pose.position.x
    position_y = sub_msg.pose.pose.position.y
    orientation_x = sub_msg.pose.pose.orientation.x
    orientation_y = sub_msg.pose.pose.orientation.y
    orientation_z = sub_msg.pose.pose.orientation.z
    orientation_w = sub_msg.pose.pose.orientation.w

    (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                            orientation_y, orientation_z, orientation_w],
                            'sxyz')
    # make the zero reference:
    global first_time_subscriber
    global position_x0
    global position_y0
    global yaw0
    global printing_stuff
    
    # print ("1------"+str(first_time_subscriber))
    if first_time_subscriber:
        position_x0 = position_x
        position_y0 = position_y
        yaw0 = yaw
        first_time_subscriber = False
        # print ("2------"+str(first_time_subscriber))

    position_x1 = position_x -position_x0
    position_y1 = position_y -position_y0
    yaw1 = yaw -yaw0

    # print_to_terminal

    # rospy.loginfo("x = {:.2f}[m], y = {:.2f}[m], theta_z = {:.1f}[degrees]".format(position_x1, position_y1, yaw1))
    # rospy.loginfo(f"===========================")
    rate.sleep()
    printing_stuff = printing_stuff +1
    if (printing_stuff%30 == 0):
        # rospy.loginfo(f'{printing_stuff}-------sadsdsdsdsd')
        rospy.loginfo("x = {:.2f}[m], y = {:.2f}[m], theta_z = {:.1f}[degrees]".format(position_x1, position_y1, yaw1))
        rospy.loginfo(f"===========================")

# def print_to_terminal ():
#     rospy.loginfo("x = {:.2f}[m], y = {:.2f}[m], theta_z = {:.1f}[degrees]".format(position_x1, position_y1, yaw1))
#     rospy.loginfo(f"===========================")
#     rate.sleep()

    

def shutdownhook():

    rospy.loginfo("==========")
    rospy.loginfo("STOP the subscriber...")



rospy.init_node(node_name, anonymous=True)
sub = rospy.Subscriber(topic_name, Odometry, callback_function)
rate = rospy.Rate(30)
rospy.on_shutdown(shutdownhook)
rospy.loginfo(f"The '{node_name}' node is active...")
rospy.spin()



