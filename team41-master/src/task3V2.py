#!/usr/bin/env python3

import rospy
from mytb3 import Tb3Move, Tb3Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
from enum import Enum

class States(Enum):
    TURNleft = 1
    TURNright = 2
    TOleft = 3
    TOright = 4
    WALL_FOLLOW = 5
    STOP = 6

class wall_follower:

   
    ranges = None

    def __init__(self):

        rospy.init_node('wall_follower')
        self.rate = rospy.Rate(10)
        self.robot_controller =Tb3Move()
        self.odom = Tb3Odometry()

        # Subscribe to the LIDAR data
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.on_shutdown(self.shutdown_hook)
        self.ctrl_c = False 


    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges
        self.rate.sleep()


    def shutdown_hook(self):
        rospy.loginfo('==== The robot STOP ====')
        self.robot_controller.stop()
        self.ctrl_c =True


    def judgement(self, rx): 
        
        fr = rx[-54]
        fl = rx[54]
        f = rx[0]
        l = rx[90]
        r = rx[-90]
        b = rx[180]

        alpha = 0.42
        twosides_Threshold = 0.7
        front_Threshold = 0.6
        back_Threshold = 0.8

        rx_prime = np.array(rx)
        close_count = float(np.count_nonzero(rx_prime<=back_Threshold)) / 360.0 * 100.0

        if (close_count > 75 and f <= front_Threshold and fr <= alpha * 1.5):
            return States.STOP

        elif (f <= front_Threshold and l <= twosides_Threshold and r <= twosides_Threshold and fl <= alpha * 1.5 and fr <= alpha * 1.5):
            return States.STOP

        elif (f <= front_Threshold and b <= back_Threshold and r <= twosides_Threshold and fl <= alpha * 1.5 and fr <= alpha * 1.5):
            return States.STOP

        elif (r <= twosides_Threshold and fr >= alpha * 1.5): 
            return States.TOright

        elif (f <= front_Threshold and l <= twosides_Threshold and r <= twosides_Threshold and fl >= alpha * 1.5 and fr <= alpha * 1.5):
            return States.TOleft

        elif (r >= twosides_Threshold and fr >= alpha * 1.5):
            return States.TURNright

        elif (f <= front_Threshold and l >= twosides_Threshold and r <= twosides_Threshold and fl >= alpha * 1.5 and fr <= alpha * 1.5):
            return States.TURNleft

        else:
            return States.WALL_FOLLOW

    def main(self):

        t = rospy.get_rostime()
        while rospy.get_rostime().secs - t.secs < 0.5:
            continue

        while not (self.ranges):
            self.rate.sleep()
        # while not self.ranges.any():
        #     continue

        while not self.ctrl_c:

            state = self.judgement(self.ranges)

            if (state == States.WALL_FOLLOW):
                fr = self.ranges[-54]
                e = 0.42 - fr
                kp = 3
                self.robot_controller.set_move_cmd(0.22, kp * e)

            elif (state == States.TOright or state == States.TURNright):
                self.robot_controller.set_move_cmd(0.22, -0.9) 

            elif (state == States.TOleft or state == States.TURNleft):
                self.robot_controller.set_move_cmd(0.22, 1.5)

            elif (state == States.STOP):
                self.robot_controller.set_move_cmd(0, 1.5)

            else:
                self.robot_controller.set_move_cmd(0, 0)
            
            self.robot_controller.publish()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        wall_follower().main()
    except rospy.ROSInterruptException:
        pass