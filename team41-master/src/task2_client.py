#!/usr/bin/env python3
import rospy
import actionlib
from math import radians, sqrt 
from mytb3 import Tb3Move
import datetime as date_t

from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback
from sensor_msgs.msg import LaserScan

from random import randint, random, choice
import numpy as np


class ObstacleAvoid_client(object):

    def __init__(self):

        self.action_server_name = "/obstacle_avoidance_actionServer"
        # self.action_client = actionlib.SimpleActionClient(self.action_server_name, SearchAction)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.subsrciber_callback)
        self.robot_controller = Tb3Move()
        # make an instance of the SearchGoal:
        self.goal = SearchGoal()
        self.complete = False 
        self.rate = rospy.Rate(1)
        self.action_client = actionlib.SimpleActionClient(self.action_server_name, SearchAction)
        self.action_client.wait_for_server()
        self.threshold = 0.6
        self.threshold_outCorner = 0.2
        # self.travelled_dis = 0 
        rospy.on_shutdown(self.shutdownhook)

    # callback function for action server:


    def feedback_callback(self, feedback_data: SearchFeedback):
        rospy.loginfo(f'=======The distance travelled: {feedback_data.current_distance_travelled}=======')
        # self.travelled_dis = feedback_data.current_distance_travelled

        # pass
    # callback function for subscriber
    def subsrciber_callback(self, lazer_data: LaserScan):

        self.ranges_callback = lazer_data.ranges
        # produce two bencons to make the robot stop spinning 
        self.leftbencon_stopTurning = np.array(self.ranges_callback[0:21]).min()
        self.rightbencon_stopTurning = np.array(self.ranges_callback[-20:]).min()
       

        left_arc = lazer_data.ranges[0:21]
        right_arc = lazer_data.ranges[-20:]
    
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.bencon_outCorner = np.array(self.ranges_callback[0:100] + self.ranges_callback[260:359]).min()
        self.max_dis_left = np.array(self.ranges_callback[0:89]).max()
        self.max_dis_right = np.array(self.ranges_callback[270:359]).max()
        # self.angle_out_ofCorner = 

    def send_goal(self, velocity, approach):
        # fwd_velocity : The speed at which the robot should move forwards (m/s)
        # approach_diatance : LaserScan distance to trigger the robot to stop (meters) (threhold dis)
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach
        # send the goal to the action server:
        self.action_client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        
    def main(self):

        t = rospy.get_rostime()
        while rospy.get_rostime().secs - t.secs < 0.3:
            continue
        
        while not self.front_arc.any():
            continue

        # !Here add something for first time move
        self.robot_controller.set_move_cmd(0.0, 0.9)
        self.robot_controller.publish()

        while (self.front_arc.min() <= 1.5):
            continue
        self.robot_controller.stop()

        while (True):
            self.complete = False
            self.send_goal(0.26, 0.35)
            self.action_client.wait_for_result()
            # A variable stored the result from the server:
            self.result = self.action_client.get_result()
            # A variable stored the closest_object from the sever
            #'self.result.closest_object_angle' stored the cloest object angle between -20---20:
            self.closest_object_angle = self.result.closest_object_angle
            # print (f'AAAAA{self.closest_object_angle}---')
            self.start_spining()
            self.complete = True

    # important :
    def start_spining(self):
        # self.robot_controller.set_move_cmd(0.0, 1.5)
        # self.robot_controller.publish()

        if self.max_dis_left >= self.threshold and self.max_dis_right >= self.threshold:
            if self.closest_object_angle <= 0: 
                rospy.loginfo('====CLOSEST object angle: LEFT==SO turn right')
                angular_z = -1.5
            else:
                rospy.loginfo('====CLOSEST object angle: RIGHT==SO turn left')
                angular_z = 1.5
        else:
            if self.max_dis_right >= self.max_dis_left:
                # to right:
                rospy.loginfo('====LEFT SO TURN RIGHT====')
                angular_z = -1.5
            else:
                # to left:
                rospy.loginfo('====RIGHT SO TURN LEFT====')
                angular_z = 1.5

        self.outofCorner()
        # rospy.loginfo('MOVE!!!!!')      
        self.robot_controller.set_move_cmd(0.0, angular_z)
        self.robot_controller.publish()

        if angular_z < 0: 
            # while self.leftbencon_stopTurning <= self.threshold:
            while self.front_arc.min() < self.threshold:
                continue
        if angular_z > 0:
            # while self.rightbencon_stopTurning >= self.threshold:
            while self.front_arc.min() < self.threshold:
                continue
            
        self.robot_controller.stop()

    def outofCorner(self):

        corner_threshold = 0.27
        min_dist = np.array(self.ranges_callback[0:100] + self.ranges_callback[260:359]).min()
        if min_dist <= corner_threshold: 
            self.robot_controller.set_move_cmd(-0.15, 0)
            self.robot_controller.publish()
            s = rospy.get_rostime()
            while rospy.get_rostime().secs - s.secs < 1:
                continue
            self.robot_controller.stop()
            
        
    def shutdownhook(self):
        # cancel the server here: so should write some codes to stop the bot 
        # in the "cancel" part of the actiob server.
        rospy.logwarn("the client will be shutdowned ... ")
        self.action_client.cancel_goal()
        rospy.loginfo("===GOALS CANCEL===")
        self.robot_controller.stop()




if __name__ == '__main__':

    node_name = 'ObstacleAvoid_client'
    rospy.init_node(node_name)
    ObstacleAvoid_client().main()
    rospy.spin()

