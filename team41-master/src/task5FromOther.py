#! /usr/bin/env python3

import rospy
import actionlib

from mytb3 import Tb3Move, Tb3Odometry
from sensor_msgs.msg import LaserScan

from cv_bridge import CvBridge, CvBridgeError

from math import cos, sin, atan, pi, sqrt

import numpy as np
from enum import Enum

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class wall_follower:
   
    closest_object_angle = None
    distance_travelled = 0
    ranges = None
    current_colour = None
    current_colour_x = None
    current_colour_y = None
    camera_width = None
    current_colour_size = None
    colours = ["red", "yellow", "green", "turquoise", "blue", "purple"]
    thresholds = {
        "red": {
            "lower": (-3, 180, 100),
            "upper": (3, 255, 255)
        },
        "yellow": {
            "lower": (25, 130, 100),
            "upper": (31, 255, 255)
        },
        "green": {
            "lower": (57, 150, 100),
            "upper": (61, 255, 255)
        },
        "turquoise": {
            "lower": (83, 134, 100),
            "upper": (93, 255, 255)
        },
        "blue": {
            "lower": (114, 218, 100),
            "upper": (123, 255, 255)
        },
        "purple": {
            "lower": (145, 165, 100),
            "upper": (152, 255, 255)
        }
    }


    def __init__(self):


        rospy.init_node('wall_follower')
        self.rate = rospy.Rate(10)
        self.robot_controller = Tb3Move()
        self.odom = Tb3Odometry()
        self.space_checker_thershold = 1
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.camera = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        rospy.on_shutdown(self.shutdown_ops)
        self.main() 

    def camera_callback(self, img_data):

        try:

            cvbridge_interface = CvBridge()

            try:
                cv_img_original = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)

            height, width, channels = cv_img_original.shape

            # crop the image
            crop_height = 400
            crop_z0 = int((height / 2) - (crop_height / 2))
            cv_img = cv_img_original[crop_z0:crop_z0+crop_height, :]

            height, width, channels = cv_img.shape
            self.camera_width = width

            colour = None
            x = None
            y = None
            current_colour_size = None
            for c in self.colours:
                hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
                img_mask = cv2.inRange(hsv_img, self.thresholds[c]["lower"], self.thresholds[c]["upper"])
                filtered_img = cv2.bitwise_and(cv_img, cv_img, mask = img_mask)
                is_black = np.all([filtered_img == 0])
                if not is_black:
                    colour = c

                    m = cv2.moments(img_mask)
                    x = int(m['m10'] / (m['m00'] + 1e-5))
                    y = int(m['m01'] / (m['m00'] + 1e-5))

                    size = (img_mask>0).mean() * 100
                    current_colour_size = size
                    break
            
            self.current_colour = colour
            self.current_colour_x = x
            self.current_colour_y = y
            self.current_colour_size = current_colour_size

        except:

            print("Error during camera processing, waiting for next frame...")


   
    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges
        self.space_checker = self.ranges[15]

    
    def shutdown_ops(self):
        rospy.loginfo("Received a shutdown request.")
        self.robot_controller.stop()


    def angle_diff(self, u, v):
        a = v - u
        a = (a + 180) % 360 - 180
        return round(a)


    def calculate_state_followLeftwall(self, rx):
        fr = np.array(rx[-57:-53]).min()
        fl = np.array(rx[53:57]).min()
        # fr = rx[-55]
        # fl = rx[55]
        f = rx[0]
        l = rx[90]
        r = rx[-90]
        b = rx[180]

        alpha = 0.38 
        side_threshold = 0.7
        front_threshold = 0.6
        back_threshold = 0.8

        rx_prime = np.array(rx)
        close_count = float(np.count_nonzero(rx_prime<=back_threshold)) / 360.0 * 100.0

        if (close_count > 75 and f <= front_threshold and fl <= alpha * 1.5):
            return State.STOP
        elif (f <= front_threshold and l <= side_threshold and r <= side_threshold and fl <= alpha * 1.5 and fr <= alpha * 1.5):
            return State.STOP
        elif (f <= front_threshold and b <= back_threshold and l <= side_threshold and fl <= alpha * 1.5 and fr <= alpha * 1.5):
            return State.STOP
        elif (l <= side_threshold and fl >= alpha * 1.5): 
            return State.TO_LEFT
        elif (f <= front_threshold and l <= side_threshold and r <= side_threshold and fr >= alpha * 1.5 and fl <= alpha * 1.5):
            return State.TO_RIGHT

        elif (l >= side_threshold and fl >= alpha * 1.5):
            return State.LEFT_TURN

        elif (f <= front_threshold and r >= side_threshold and l <= side_threshold and fr >= alpha * 1.5 and fl <= alpha * 1.5):
            return State.RIGHT_TURN
        else:
            return State.WALL_FOLLOW

        

    def calculate_state(self, rx): 
        
        fr = np.array(rx[-57:-53]).min()
        fl = np.array(rx[53:57]).min()
        # fr = rx[-55]
        # fl = rx[55]
        f = rx[0]
        l = rx[90]
        r = rx[-90]
        b = rx[180]

        alpha = 0.38 # the desired location
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
            return State.TO_RIGHT
        elif (f <= front_threshold and l <= side_threshold and r <= side_threshold and fl >= alpha * 1.5 and fr <= alpha * 1.5):
            return State.TO_LEFT
        elif (r >= side_threshold and fr >= alpha * 1.5):
            return State.RIGHT_TURN
        elif (f <= front_threshold and l >= side_threshold and r <= side_threshold and fl >= alpha * 1.5 and fr <= alpha * 1.5):
            return State.LEFT_TURN
        else:
            return State.WALL_FOLLOW

    def get_start_colour(self):

        # variables
        yaw_init = self.odom.yaw
        speed = -0.8

        # turn 180 degrees
        self.robot_controller.set_move_cmd(0, speed)
        while self.angle_diff(yaw_init, self.odom.yaw) <= 0: # switch to positive indicates 180 degrees
            self.robot_controller.publish()

        # record colour
        start_colour = self.current_colour

        # turn next 180 degrees
        while self.angle_diff(yaw_init, self.odom.yaw) >= 15: # switch to negative indicates a full 360 rotation (15 degree stopping time)
            self.robot_controller.publish()
        self.robot_controller.stop()
        self.colours = [start_colour]

        rospy.sleep(1) # sleep a bit

        return start_colour

    def main(self, recursive_call = False):

        self.beacon_threshold = 2.5

        if not recursive_call:

            while not (self.ranges):
                self.rate.sleep()

            self.start_colour = self.get_start_colour()
            print ("TARGET COLOUR DETECTED: The target beacon is " + str(self.start_colour) + ".")
            self.x_init, self.y_init = self.odom.posx, self.odom.posy

            # rospy.loginfo(f'----------{self.space_checker}')

            if self.space_checker < self.space_checker_thershold:
                self.while_loopByRightWall()
                
            
            elif self.space_checker >= self.space_checker_thershold:
                self.while_loopByLeftWall()

        print("TARGET BEACON IDENTIFIED: Beaconing initiated.")
        self.beacon(self.start_colour)


    def while_loopByLeftWall(self):
         while True:
            color = self.start_colour
            d = sqrt((self.odom.posx - self.x_init)**2 + (self.odom.posy - self.y_init)**2)
            if self.current_colour == self.start_colour and d > self.beacon_threshold and self.current_colour_size > 15:
                break
            state = self.calculate_state_followLeftwall(self.ranges)
            # HERE!!!!!!!!!!!!!!!!!!!
            if (state == State.WALL_FOLLOW):
                fl = self.ranges[55]
                e = 0.38 - fl
               
                kp = 2.6
                self.robot_controller.set_move_cmd(0.24, -(kp * e))

            elif (state == State.TO_RIGHT or state == State.RIGHT_TURN):
                self.robot_controller.set_move_cmd(0.25, -1.52) # 4/5/21 1 0.2 0.8
            elif (state == State.TO_LEFT or state == State.LEFT_TURN):
                self.robot_controller.set_move_cmd(0.24, 0.86)
            elif (state == State.STOP):
                self.robot_controller.set_move_cmd(0, -1.6)
            else:
                self.robot_controller.set_move_cmd(0, 0)
            
            self.robot_controller.publish()
            self.rate.sleep()
      


    def while_loopByRightWall(self):
        while True:
             # Break to beacon
            color = self.start_colour
            d = sqrt((self.odom.posx - self.x_init)**2 + (self.odom.posy - self.y_init)**2)
            if self.current_colour == self.start_colour and d > self.beacon_threshold and self.current_colour_size > 15:
                # print("Should start beaconing usually!")
                break
            
            state = self.calculate_state(self.ranges)
            # else:
            #     state = self.calculate_state_followLeftwall(self.ranges)

            if (state == State.WALL_FOLLOW):
                fr = self.ranges[-55]
                e = 0.38 - fr
                kp = 3
                self.robot_controller.set_move_cmd(0.25, kp * e)
            elif (state == State.TO_RIGHT or state == State.RIGHT_TURN):
                self.robot_controller.set_move_cmd(0.25, -0.9) # 4/5/21 1 0.2 0.8
            elif (state == State.TO_LEFT or state == State.LEFT_TURN):
                self.robot_controller.set_move_cmd(0.25, 1.5)
            elif (state == State.STOP):
                self.robot_controller.set_move_cmd(0, 1.5)
            else:
                self.robot_controller.set_move_cmd(0, 0)
            
            self.robot_controller.publish()
            self.rate.sleep()


  

    def beacon(self, colour):

        # constants
        threshold = 0.3
        obstacle_threshold = 0.7
        min_loc = (self.camera_width / 2) - (self.camera_width * 0.01)
        max_loc = (self.camera_width / 2) + (self.camera_width * 0.01)
        x_speed = 0.15
        z_speed = 0.25

        x = self.current_colour_x

        # turn to face beacon
        while not (x < 1.25 * max_loc and x > 0.75 * min_loc):
            x = self.current_colour_x
            if x < min_loc: # turn left
                self.robot_controller.set_move_cmd(0, z_speed)
            elif x > max_loc: # turn right 
                self.robot_controller.set_move_cmd(0, -z_speed)
            else: # go straight
                self.robot_controller.set_move_cmd(0, 0)
            self.robot_controller.publish()
            self.rate.sleep()

        self.robot_controller.stop()
        rospy.sleep(0.5)

        # general beaconing
        while self.current_colour_size < 45: 

            # print("S:" + str(self.current_colour_size))

            if (self.current_colour != colour):
                print("Giving up on beacon, returning to wall follower")
                self.main(True)
                return

            # get lidar and camera information
            x = self.current_colour_x
            y = self.current_colour_y
            fl = np.array(self.ranges[0:35]).min()
            fr = np.array(self.ranges[-36:]).min()

            # adjust due to obstacles 
            if (fl <= obstacle_threshold or fr <= obstacle_threshold):
                # print("adjust due to obstacle")
                if (fl < fr): # turn right
                    self.robot_controller.set_move_cmd(x_speed, -z_speed)
                else: # turn left
                    self.robot_controller.set_move_cmd(x_speed, z_speed)
            
            # adjust to beacon
            else:
                # print("adjust due to beacon")
                if x < min_loc: # turn left
                    self.robot_controller.set_move_cmd(x_speed, z_speed)
                elif x > max_loc: # turn right 
                    self.robot_controller.set_move_cmd(x_speed, -z_speed)
                else: # go straight
                    self.robot_controller.set_move_cmd(x_speed, 0)

            self.robot_controller.publish()
            self.rate.sleep()

        self.robot_controller.stop()

        print("final approach")

        # final approach (up to 100% and 0.4m)
        while np.array(self.ranges[-20:] + self.ranges[0:20]).min() >= threshold:
            if x < min_loc: # turn left
                self.robot_controller.set_move_cmd(0.1, 0.1)
            elif x > max_loc: # turn right 
                self.robot_controller.set_move_cmd(0.1, -0.1)
            else: # go straight
                self.robot_controller.set_move_cmd(0.1, 0)
            self.robot_controller.publish()
            self.rate.sleep()

        self.robot_controller.stop()
        print("FINAL CHALLENGE COMPLETE: The robot has now stopped.")

class State(Enum):
    LEFT_TURN = 1
    RIGHT_TURN = 2
    TO_LEFT = 3
    TO_RIGHT = 4
    WALL_FOLLOW = 5
    STOP = 6

if __name__ == '__main__':
    try:
        wall_follower()
    except rospy.ROSInterruptException:
        pass