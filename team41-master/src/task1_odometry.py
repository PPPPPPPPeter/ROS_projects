#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi

class Circle:
    
    # The callback function for subscriber:     
    def callback_function(self, sub_msg):

        position_x = sub_msg.pose.pose.position.x
        position_y = sub_msg.pose.pose.position.y
        orientation_x = sub_msg.pose.pose.orientation.x
        orientation_y = sub_msg.pose.pose.orientation.y
        orientation_z = sub_msg.pose.pose.orientation.z
        orientation_w = sub_msg.pose.pose.orientation.w

        (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                                orientation_y, orientation_z, orientation_w],
                                'sxyz')

        self.real_positionX = position_x
        self.real_positionY = position_y
        self.real_positionZ = yaw


        if self.startup:

            self.startup = False 
            self.x0 = self.real_positionX
            self.y0 = self.real_positionY
            self.z0 = self.real_positionZ


    def __init__(self):
        
        # When setting up the publisher, the "cmd_vel" topic needs to be specified
        # and the Twist message type needs to be provided
        self.startup = True

        self.ctrl_c = False

        self.startMoveCircle = True

        self.antiCircle_completed = False  

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        rospy.init_node('move_circle', anonymous=True)
        self.rate = rospy.Rate(1) # hz
        # self.rate_pub = rospy.Rate(10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        
        rospy.on_shutdown(self.shutdownhook)
        
        self.vel_cmd = Twist()

        rospy.loginfo("===== the 'move_circle' node is ACTIVE... =====")

        # inital:
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0
        self.real_positionX = 0.0
        self.real_positionY = 0.0
        self.real_positionZ = 0.0


    def shutdownhook(self):
        
        # publish to the /cmd_vel topic to make the robot stop
        self.pub.publish(Twist())
        rospy.loginfo("========================")
        rospy.loginfo("==== STOP the robot ====")
        self.ctrl_c = True 
    

    def anticlockwiseCircle(self):

        self.vel_cmd.linear.x = 0.13
        self.vel_cmd.angular.z = 0.13/0.5
        self.pub.publish(self.vel_cmd)
        self.rate.sleep()
        i = 0
        for i in range(10):
            self.rate.sleep()
            i +1

        # self.rate.sleep()
        while not self.ctrl_c:

            if self.startMoveCircle:
                i=0
                for i in range(10):
                    self.rate.sleep()
                    i + 1
                self.startMoveCircle = False
            
            else :

                if abs(self.real_positionZ - self.z0)>0.01:
                    self.antiCircle_completed = True
                    continue
                else:
                    self.vel_cmd = Twist()
                    break
        
        self.pub.publish(self.vel_cmd)
        if self.antiCircle_completed:

            self.clockwiseCircle()
        else:

            self.anticlockwiseCircle()

    
        

    
    def clockwiseCircle(self):

        self.startMoveCircle = True
        self.vel_cmd.linear.x = 0.13
        self.vel_cmd.angular.z = -(0.13/0.5)
        self.pub.publish(self.vel_cmd)
        self.rate.sleep()

        while not self.ctrl_c:

            if self.startMoveCircle:
                i=0
                for i in range(10):
                    self.rate.sleep()
                    i + 1
                self.startMoveCircle = False
            
            else :

                if abs(self.real_positionZ - self.z0)>0.01:
                    continue
                else:
                    self.vel_cmd = Twist()
                    break
        
        self.pub.publish(self.vel_cmd)
           


    def main_loop(self):

        i=0
        for i in range(4):
            self.rate.sleep()
            i + 1

        self.anticlockwiseCircle()
        self.pub.publish(self.vel_cmd)

if __name__ == '__main__':
    vel_ctlr = Circle()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass
