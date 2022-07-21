#!/usr/bin/env python3
import rospy
import actionlib
from mytb3 import Tb3LaserScan, Tb3Move, Tb3Odometry
from math import radians, sqrt 
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction,SearchGoal
from sensor_msgs.msg import LaserScan
import numpy as np 
# CHANGE the middle function !!!!!!!!!!!!!!!!! 

# action server will make the robot move and provide the feedback to client and also the result:
class ObstacleAvoid_server(object):

    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer("/obstacle_avoidance_actionServer",
         SearchAction, self.action_server_launch, auto_start=False)
        self.action_server.start()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.subsrciber_callback)

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        # self.threshold = 1.5
        self.rate = rospy.Rate(10)
        self.success = False


    def subsrciber_callback(self, lazerScan_data:LaserScan):
        # create the datas in front of the robot :
        left_arc = lazerScan_data.ranges[0: 21]
        right_arc = lazerScan_data.ranges[-20:]
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        # determine the minimum distance of the ranges of front_arcs:
        self.min_dis = self.front_arc.min()
        arc_angles = np.arange(-20, 21)
        # determine the location of the angle of the min_distance:
        # in the ranges of the front of the robot:
        self.object_angle = arc_angles[np.argmin(self.front_arc)]


    # The callback function of the action server use to recieved the goal from the client:

    def action_server_launch(self, goal_data: SearchGoal):

        
        # check the goal from the action client:
        # firstly set the 'success' label to True: 
        self.success = True
        if goal_data.fwd_velocity <= 0 or goal_data.fwd_velocity > 0.26:
            print ('Invalid value, set the velocity between 0 and 0.26')
            self.success = False
        if goal_data.approach_distance <= 0.2:
            print("Invalid value, I'll crash!")
            self.success = False
        elif goal_data.approach_distance > 3.5:
            print("Invalid value, I can't measure that far.")
            self.success = False
        
        if not self.success:
            self.action_server.set_aborted()
            # Stop processing here!
            return 

        move_velocity = goal_data.fwd_velocity
        stop_distance = goal_data.approach_distance
        # using two variables to record the initial position:
        x_init = self.robot_odom.posx
        y_init = self.robot_odom.posy

        rospy.loginfo(f'==The robot will move forward at {move_velocity} m/s and stop after {stop_distance} m==')
        self.robot_controller.set_move_cmd(move_velocity, 0.0)
        # if the minimun distance always > stop_distance, the robot safely move else will stop:
        # rospy.loginfo('====1111111111111111111111111111====')
        # if ((self.min_dis - 0.02) >= stop_distance):
        #     rospy.loginfo('====TRUE!!!====')
        # else:
        #     rospy.loginfo('====FALSE!!!====')

        while (self.min_dis - 0.08) >= stop_distance:
            # rospy.loginfo('=====33333333333333333333333333=====')
            # start moving:
            self.robot_controller.publish()
            self.rate.sleep()
            # if the server is cancelled:
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                rospy.loginfo('=== The goal has been CANCELLED ... ===')
                rospy.loginfo('=== The robot will also been STOP ... ===') 
                self.robot_controller.stop()
                self.success = False
                break
            # publish the feedback of the real distance of robot to client:
            travelled_dis = sqrt((self.robot_odom.posx - x_init)**2+(self.robot_odom.posy - y_init)**2)
            self.feedback.current_distance_travelled = self.robot_odom.round(travelled_dis, 2)
            self.action_server.publish_feedback(self.feedback)
        
        # self.robot_controller.set_move_cmd(-0.18, 0.0)
        # self.robot_controller.publish()
        # t1 = rospy.get_rostime()
        # while (rospy.get_rostime().secs - t1.secs < 0.5):
        #     continue
        # !!!!!!!!!!!!!!!!HERE!!!!!!
        # self.robot_controller.stop()
        # self.robot_controller.set_move_cmd(0.0, -0.5)
        # while (self.front_arc.min()<= self.threshold ):
        #     continue
        # self.robot_controller.set_move_cmd(0.0, 0.9)
        # self.robot_controller.publish()
        # t = rospy.get_rostime()
        # while (rospy.get_rostime().secs - t.secs) < 1.3:
        #     continue
        # while (self.front_arc.min() <= self.threshold):
        #     continue
        # self.robot_controller.stop()
        

        
        # if finish one loop goal so need to send the result to the client:
        if self.success:
            rospy.loginfo('==== A Request FINISH ====')
            travelled_dis = sqrt((self.robot_odom.posx - x_init)**2+(self.robot_odom.posy - y_init)**2)
            self.result.total_distance_travelled = self.robot_odom.round(travelled_dis, 2)
            self.result.closest_object_angle = self.object_angle
            self.result.closest_object_distance = self.min_dis
            self.action_server.set_succeeded(self.result)
            self.robot_controller.stop()


    
if __name__ == '__main__':

    node_name = 'ObstacleAvoid_server'
    rospy.init_node(node_name)
    ObstacleAvoid_server()
    rospy.spin()

