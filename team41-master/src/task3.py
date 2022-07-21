#!/usr/bin/env python3

from pickle import STOP
import rospy
import actionlib

# from move_tb3 import MoveTB3
# from tb3_odometry import TB3Odometry
from mytb3 import Tb3LaserScan, Tb3Move, Tb3Odometry
from sensor_msgs.msg import LaserScan

from math import cos, sin, atan, pi

import numpy as np
from enum import Enum

class wall_follower:
   
    ranges = None

    def __init__(self):

        # Create node
        rospy.init_node('wall_follower')

        # Set refresh rate
        self.rate = rospy.Rate(10)

        # Create movement controller
        self.robot_controller =Tb3Move()
        self.odom = Tb3Odometry()

        # Subscribe to the LIDAR data
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Setup shutdown hooks
        rospy.on_shutdown(self.shutdown_hook)
        self.ctrl_c = False 
        # Setup main loop
        self.main() 

    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges


    def shutdown_hook(self):
        rospy.logwarn("Received a shutdown request.")
        self.robot_controller.stop()
        self.ctrl_c =True


    def calculate_state(self, rx): 
        
        fr = rx[-55]
        fl = rx[55]
        f = rx[0]
        l = rx[90]
        r = rx[-90]
        b = rx[180]

        alpha = 0.42
        side_threshold = 0.7
        front_threshold = 0.6
        back_threshold = 0.8

        rx_prime = np.array(rx)
        close_count = float(np.count_nonzero(rx_prime<=back_threshold)) / 360.0 * 100.0

        if (close_count > 75 and f <= front_threshold and fr <= alpha * 1.5):
            return State.STOP
        elif (f <= front_threshold and l <= side_threshold and r <= side_threshold and fl <= alpha * 1.5 and fr <= alpha * 1.5):
            return State.STOP
        elif (f <= front_threshold and b <= back_threshold and r <= side_threshold and fl <= alpha * 1.5 and fr <= alpha * 1.5):
            return State.STOP
        elif (r <= side_threshold and fr >= alpha * 1.5): 
            return State.TOright
        elif (f <= front_threshold and l <= side_threshold and r <= side_threshold and fl >= alpha * 1.5 and fr <= alpha * 1.5):
            return State.TOleft
        elif (r >= side_threshold and fr >= alpha * 1.5):
            return State.TURNright
        elif (f <= front_threshold and l >= side_threshold and r <= side_threshold and fl >= alpha * 1.5 and fr <= alpha * 1.5):
            return State.TURNleft
        else:
            return State.WALL_FOLLOW

    def main(self):

        # Wait till we receive readings
        while not (self.ranges):
            self.rate.sleep()

        while not self.ctrl_c:

            state = self.calculate_state(self.ranges)

            if (state == State.WALL_FOLLOW):
                fr = self.ranges[-55]
                e = 0.42 - fr
                kp = 3
                self.robot_controller.set_move_cmd(0.26, kp * e)
            elif (state == State.TOright or state == State.TURNright):
                self.robot_controller.set_move_cmd(0.26, -0.9) 
            elif (state == State.TOleft or state == State.TURNleft):
                self.robot_controller.set_move_cmd(0.26, 1.5)
            elif (state == State.STOP):
                self.robot_controller.set_move_cmd(0, 1.5)
            else:
                self.robot_controller.set_move_cmd(0, 0)
            
            self.robot_controller.publish()
            self.rate.sleep()

class State(Enum):
    TURNleft = 1
    TURNright = 2
    TOleft = 3
    TOright = 4
    WALL_FOLLOW = 5
    STOP = 6

if __name__ == '__main__':
    try:
        wall_follower()
    except rospy.ROSInterruptException:
        pass