#! /usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry 
from tb3 import Tb3Move
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import Twist
from math import inf, pi
from pathlib import Path
import roslaunch


class colour_search(object):

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub1 = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback)

        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.rate = rospy.Rate(10) # hz
        self.fast = -0.5
        self.slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(10)
        
        self.vel_cmd = Twist()

        self.f_dist = 10
        self.directly_ahead_threshold = 15
        self.left_distance = 0
        self.right_distance = 10
        self.front_ahead_dist = 0.45
        self.side_ahead_dist = 0.5
        self.fl_max_dist = 0
        self.fl_max_dist = 0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0
        self.startup = True

        self.left_wall = False
        self.right_wall = False 
        self.front_wall = False

        self.ctrl_c = False
        self.locate_l_wall = False
        self.locate_wall = False
        self.locate_init_dist = False 
        self.init_dist = 0
        self.fl_dist = 10
        self.fr_dist = 10
        self.br_dist = 15
        self.bl_dist = 15

        self.base_image_path = Path.home()

    

    def callback(self, topic_message):
        x_direction = topic_message.pose.pose.orientation.x
        y_direction = topic_message.pose.pose.orientation.y
        z_direction = topic_message.pose.pose.orientation.z
        w_direction = topic_message.pose.pose.orientation.w

        x_pos = topic_message.pose.pose.position.x
        y_pos = topic_message.pose.pose.position.y
        z_pos = topic_message.pose.pose.position.z

        (roll, pitch, yaw) = euler_from_quaternion([x_direction, 
                                y_direction, z_direction, w_direction],
                                'sxyz')
        self.x = x_pos
        self.y = y_pos
        self.z = yaw     

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.z0 = self.z   

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def callback_lidar(self, lidar_data):
        f = np.array(lidar_data.ranges[0:35] + lidar_data.ranges[-35:])
        f_ahead = np.array(lidar_data.ranges[0])
        l = np.array(lidar_data.ranges[85])

        fr =  np.array(lidar_data.ranges[330 : 355])      
        fl = np.array(lidar_data.ranges[0:35]) 

        arc_br = np.array(lidar_data.ranges[195 : 225])
        arc_bl = np.array(lidar_data.ranges[135 : 165])
        
        r = np.array(lidar_data.ranges[-90])


        self.f_dist = f.mean()
        self.directly_ahead_threshold = f_ahead.mean()
        self.left_distance = l.mean()
        self.right_distance = r.mean()
        self.fl_dist = fl.mean()
        self.fr_dist = fr.mean()
        self.br_dist = arc_br.mean()
        self.bl_dist = arc_bl.mean()
        
    
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
    


    def get_left_angle(self, angle, moveBy):
        negative = False
        if angle < 0:
            negative = True
        if angle + moveBy > 180:
            new_angle = 180 - (moveBy - (180 - angle))
            if not negative:
                new_angle *= -1
            return new_angle
        else:
            return angle + moveBy


    def get_right_angle(self, angle, moveBy):
        current_angle = abs(angle)
        if angle - moveBy < -180:
            new_angle = 180 - (moveBy - (180 - current_angle))
            return new_angle
        else:
            return angle - moveBy


    
    def check_wall(self):
        if self.right_distance < self.side_ahead_dist:
            self.right_wall = True
            self.locate_wall = True
        else:
            self.right_wall = False
        if self.directly_ahead_threshold < self.front_ahead_dist:
            self.front_wall = True
            self.locate_wall = True
        else:
            self.front_wall = False

        if self.left_distance < self.side_ahead_dist:
            self.left_wall = True
            self.locate_wall = True
        else:
            self.left_wall = False
        
        print(self.left_wall, self.front_wall, self.right_wall)

    
    def turn_right(self):
        self.vel_cmd.linear.x = 0
        self.vel_cmd.angular.z = 0
        self.pub.publish(self.vel_cmd)
        
        while self.f_dist < self.front_ahead_dist:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = -0.3
            self.pub.publish(self.vel_cmd)
        
            while self.bl_dist < 0.22 and self.f_dist > 0.22:
                self.vel_cmd.angular.z = 0
                self.vel_cmd.linear.x = 0.1
                self.pub.publish(self.vel_cmd)

            self.vel_cmd.linear.x = 0.0
        
        self.vel_cmd.angular.z = 0
        self.pub.publish(self.vel_cmd)




    def left_turn(self):
        kv = 0.8
        kp = 0.6

        initial_z = self.z
        
        while abs(self.z * 180/pi - self.get_left_angle(initial_z * 180/pi, 90)) > 1:
            error = abs(self.z * 180/pi - self.get_left_angle(initial_z * 180/pi, 90))
            if abs(self.z * 180/pi - self.get_left_angle(initial_z * 180/pi, 90)) > 100:
                error = (180 - abs(self.z * 180/pi)) + (180 -  abs(self.get_left_angle(initial_z * 180/pi, 90)))
            while self.br_dist < 0.22 and self.f_dist > 0.22:
                self.vel_cmd.angular.z = 0.0
                self.vel_cmd.linear.x = 0.05
                self.pub.publish(self.vel_cmd)
            
            self.vel_cmd.linear.x = 0.1 *  kv * error * pi/180
            self.vel_cmd.angular.z = 1.0 * kp * error * pi/180
            
            print(self.z * 180/pi, self.get_left_angle(initial_z * 180/pi, 90), error)
            self.pub.publish(self.vel_cmd)

            
        final_angle = self.z * 180 / pi
        self.vel_cmd.linear.x = 0
        self.pub.publish(self.vel_cmd)
        return final_angle
    
    def right_turn(self):
        kp = 0.7
        initial_z = self.z
       
        while abs(self.z * 180/pi - self.get_right_angle(initial_z * 180/pi, 90)) > 1:
            self.vel_cmd.angular.z = -1.0 * kp * (abs(self.z * 180/pi - self.get_right_angle(initial_z * 180/pi, 90)) * pi/180)
            self.pub.publish(self.vel_cmd)

        self.vel_cmd.linear.x = 0
        self.pub.publish(self.vel_cmd)

    def forward(self):
        self.vel_cmd.linear.x = 0.17
        self.vel_cmd.angular.z = 0
        self.pub.publish(self.vel_cmd)

    def backwards(self):
        self.vel_cmd.linear.x = -0.17
        self.vel_cmd.angular.z = 0
        self.pub.publish(self.vel_cmd)
        


    
    
    def left_arc(self, angle):
         kp = 0.8
         StartTime = rospy.get_rostime()
         while abs(self.z * 180/pi - angle) > 3 and rospy.get_rostime().secs - StartTime.secs < 5:
            error = abs(self.z * 180/pi - angle)
            
            print(self.z * 180/pi, angle, error)
            self.vel_cmd.angular.z = 1.0 * kp * error * pi/180
            self.pub.publish(self.vel_cmd)
        
         self.vel_cmd.linear.x = 0
         self.pub.publish(self.vel_cmd)
         self.check_wall()
    
    
    def move_until_wall(self):
        self.check_wall()
        while not self.left_wall:
            while self.fl_dist < 0.4:
                self.vel_cmd.linear.x = 0
                self.vel_cmd.angular.z = -0.3
                self.pub.publish(self.vel_cmd)

            self.check_wall()
            self.vel_cmd.linear.x = 0.17
            self.vel_cmd.angular.z = 0
            self.pub.publish(self.vel_cmd)
           
        
        self.pub.publish(self.vel_cmd)
        self.check_wall()

    
    def main(self):
        threeminuStart = rospy.get_rostime()
        while (rospy.get_rostime().secs - threeminuStart.secs < 180):
            self.check_wall()

            if not self.left_wall:
                self.vel_cmd.linear.x = 0
                self.pub.publish(self.vel_cmd)
                angle = self.left_turn()
                self.move_until_wall()
                self.left_arc(angle)
            

            elif not self.front_wall:
                self.forward()

                while self.fl_dist < 0.4:
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = -0.3
                    self.pub.publish(self.vel_cmd)
                
                while self.fr_dist < 0.4:
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = 0.3
                    self.pub.publish(self.vel_cmd)

            
            
            elif self.front_wall and self.left_wall:
                self.turn_right()

            elif self.front_wall and self.left_wall and self.right_wall:
                self.backwards()
        self.savemap()
            
            
      
            
if __name__ == "__main__":
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass