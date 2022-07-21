#!/usr/bin/env python3
import rospy
import actionlib
from math import radians, sqrt 
from mytb3 import Tb3LaserScan, Tb3Move, Tb3Odometry
import datetime as date_t

from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback
from sensor_msgs.msg import LaserScan

from random import randint, random, choice
import numpy as np
from pathlib import Path
import roslaunch
# CHANGE the 'out of corner' function !!!!!!!!!!!!!!!!! 

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
        self.threshold = 0.5
        self.threshold_outCorner = 0.15
        self.ctrl_c = False
        # self.front_arc = 0
        # self.travelled_dis = 0 
        rospy.on_shutdown(self.shutdownhook)

        self.base_image_path = Path.home()
        # self.t0 = rospy.time.now().to_sec()
        # self.t1 = 0
        # while(self.t1 - self.t0 < 5):
        #     self.t1 = rospy.Time.now().to_sec()
        # full_image_path = self.base_image_path.joinpath(f'catkin_ws/src/team41/maps/test')
        # node = roslaunch.core.Node('map_server', 'map_saver', args=f'-f {full_image_path}')

        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()
        # process = launch.launch(node)
        # process.stop

       

        # if(t1-t0 == 180):


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


    def savemap(self):

        # self.t0 = rospy.Time.now().to_sec()
        # self.t1 = 0
        # while(self.t1 - self.t0 < 5):
        #     self.t1 = rospy.Time.now().to_sec()
        full_image_path = self.base_image_path.joinpath(f'catkin_ws/src/team41/maps/task5_map')

        self.t0 = rospy.Time.now().to_sec()
        self.t1 = 0
        # while(self.t1 - self.t0 < 5):
        #     self.t1 = rospy.Time.now().to_sec()

        # full_image_path = self.base_image_path.joinpath(f'catkin_ws/src/team41/maps/task5_map')
   
        node = roslaunch.core.Node('map_server', 'map_saver', args=f'-f {full_image_path}')

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        rospy.sleep(5)
        process.stop

        
    def main(self):
        t = rospy.get_rostime()

        while rospy.get_rostime().secs - t.secs < 0.5:
            continue
        # print("AAAAAAAAAAAAAAAAAA")
        # print(self.front_arc)
        while not self.front_arc.any():
            continue
       
        self.t1 = rospy.Time.now().to_sec()
        while rospy.get_rostime().secs - t.secs < 0.5:
            continue
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Here add something for first time move
        self.robot_controller.set_move_cmd(0.0, 0.9)
        self.robot_controller.publish()

        while (self.front_arc.min() <= 0.5):
            continue
        self.robot_controller.stop()

        threeminuStart = rospy.get_rostime()

        while (rospy.get_rostime().secs - threeminuStart.secs < 180):
            print("BBBBBBB")
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
    
        print("CCCCCCCCC")
        self.savemap()
        

    # important :
    def start_spining(self):
        # self.robot_controller.set_move_cmd(0.0, 1.5)
        # self.robot_controller.publish()

        if self.max_dis_left >= self.threshold and self.max_dis_right >= self.threshold:
            if self.closest_object_angle <= 0: 
                rospy.loginfo('====CLOSEST object angle: LEFT==SO turn right')
                angular_z = -1.2
            else:
                rospy.loginfo('====CLOSEST object angle: RIGHT==SO turn left')
                angular_z = 1.2
        else:
            if self.max_dis_right >= self.max_dis_left:
                # to right:
                rospy.loginfo('====LEFT SO TURN RIGHT====')
                angular_z = -1.2
            else:
                # to left:
                rospy.loginfo('====RIGHT SO TURN LEFT====')
                angular_z = 1.2

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
        if self.bencon_outCorner <= self.threshold_outCorner:
            rospy.loginfo('OUT OF CORNER')
            # back:
            self.robot_controller.set_move_cmd(-0.18, 0.0)
            self.robot_controller.publish()
            t1 = rospy.get_rostime()
            while (rospy.get_rostime().secs - t1.secs < 0.5):
                continue

            self.robot_controller.stop()

            if self.closest_object_angle <= 0: 
                rospy.loginfo('====CORNER object angle: LEFT==SO turn right')
                angular_z = -1.5
            else:
                rospy.loginfo('====CORNER: RIGHT==SO turn left')
                angular_z = 1.5

            self.robot_controller.set_move_cmd(0.0, angular_z)
            self.robot_controller.publish()
            # t = rospy.get_rostime()
            # while (rospy.get_rostime().secs - t.secs) < 1.3:
            #     continue
            while (self.front_arc.min() <= self.threshold):
                continue
            self.robot_controller.stop()
            
        
    def shutdownhook(self):
        # cancel the server here: so should write some codes to stop the bot 
        # in the "cancel" part of the actiob server.
        rospy.logwarn("the client will be shutdowned ... ")
        self.action_client.cancel_goal()
        rospy.loginfo("===GOALS CANCEL===")
        self.robot_controller.stop()
        self.ctrl_c = True




if __name__ == '__main__':

    node_name = 'ObstacleAvoid_client'
    rospy.init_node(node_name)
    ObstacleAvoid_client().main()
    rospy.spin()

