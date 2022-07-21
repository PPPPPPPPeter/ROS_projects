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

        rospy.loginfo("x = {:.2f}[m], y = {:.2f}[m], theta_z = {:.1f}[degrees]".format(position_x, position_y, yaw))
        rospy.loginfo(f"===========================")
        self.rate.sleep()


    def __init__(self):
        
        # When setting up the publisher, the "cmd_vel" topic needs to be specified
        # and the Twist message type needs to be provided
        
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        rospy.init_node('move_circle', anonymous=True)
        self.rate = rospy.Rate(1) # hz
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        
        rospy.on_shutdown(self.shutdownhook)
        
        self.vel_cmd = Twist()

        rospy.loginfo("the 'move_circle' node is active...")

    def shutdownhook(self):
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s

        print("stopping the robot")

        # publish to the /cmd_vel topic to make the robot stop
        self.pub.publish(self.vel_cmd)


    def anticlockwiseCircle(self, state):
        self.vel_cmd.linear.x = 0.26
        self.vel_cmd.linear.y = 0
        self.vel_cmd.linear.z = 0
        self.vel_cmd.angular.x = 0
        self.vel_cmd.angular.y = 0
        self.vel_cmd.angular.z = 0.26/0.5

        # while state == 0:

        while not rospy.is_shutdown():
    
            #Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_distance = 0
    
            #Loop to move the turtle in an specified distance
            print("anticlockwise circle")
            while(current_distance < 2*pi*0.5):
                #Publish the velocity
                self.pub.publish(self.vel_cmd)
                
                self.rate.sleep()
                
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_distance= 0.26*(t1-t0)
            #After the loop, stops the robot
            # self.vel_cmd.linear.x = 0
            # self.vel_cmd.angular.z = 0

                # print("anticlockwise circle")

            print("changing state from anticlock to clock")
            state = 1
            if state == 1:
                print("finn anticlockwise, now starting clockwise")
                self.clockwiseCircle(state)

    
    def clockwiseCircle(self, state):
        
        self.vel_cmd.linear.x = 0.26
        self.vel_cmd.linear.y = 0
        self.vel_cmd.linear.z = 0
        self.vel_cmd.angular.x = 0
        self.vel_cmd.angular.y = 0
        self.vel_cmd.angular.z = -0.26/0.5
    
        # while state == 1:
        while not rospy.is_shutdown():

            #Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            
            print("clockwise circle")
            current_distance = 0
            while(current_distance < 2*pi*0.5):
                #Publish the velocity
                self.pub.publish(self.vel_cmd)
                
                self.rate.sleep()
                
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_distance= 0.26*(t1-t0)
                
                # print("clockwise circle")

                # print("changing state from clock to anticlock to stop loop")
                # state = 0
            print("finn clockwise, now stopping")
            # state = 0
            self.vel_cmd.angular.z = 0
            self.vel_cmd.linear.x = 0


    def main_loop(self):
        state = 0
        self.anticlockwiseCircle(state)
        #After the loop, stops the robot
        # self.vel_cmd.angular.z = 0
        # self.vel_cmd.linear.x = 0
            

        #Force the robot to stop
        self.pub.publish(self.vel_cmd)

if __name__ == '__main__':
    vel_ctlr = Circle()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass
