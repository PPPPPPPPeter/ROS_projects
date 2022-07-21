#!/usr/bin/env python3

from rosdep2 import RosdepDatabase
import rospy
from mytb3 import Tb3Move, Tb3Odometry
from sensor_msgs.msg import LaserScan
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
import numpy as np
from enum import Enum
from pathlib import Path
import roslaunch

class States(Enum):
    TURNleft = 1
    TURNright = 2
    TOleft = 3
    TOright = 4
    WALL_FOLLOW = 5
    STOP = 6

class wall_follower:

    closest_object_angle = None
    distance_travelled = 0
    ranges = None
    current_colour = None
    current_colour_x = None
    current_colour_y = None
    camera_width = None
    current_colour_size = None
    colours = ["red", "yellow", "green", "turquoise", "blue", "purple", "wall", "gray"]
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
        },
        "wall": {
            "lower": (0, 75, 100),
            "upper": (20, 250, 255)
        },
        "gray": {
            "lower": (0, 0, 100),
            "upper": (150, 50, 255)
        }
    }
   
    ranges = None

    def __init__(self):

        rospy.init_node('wall_follower')
        self.rate = rospy.Rate(10)
        self.robot_controller =Tb3Move()
        self.odom = Tb3Odometry()

        # Subscribe to the LIDAR data
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.camera = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)

        rospy.on_shutdown(self.shutdown_hook)
        self.ctrl_c = False 
        self.threshold = 0.5
        self.avoid_obstac_threshold = 0.3
        self.wall_exist = False
        self.first_move = True
        self.base_image_path = Path.home()

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
    
            # rospy.loginfo(f'COLOR: {colour}')
            self.current_colour = colour
            self.current_colour_x = x
            self.current_colour_y = y
            self.current_colour_size = current_colour_size

        except:

            print("Error during camera processing, waiting for next frame...")


    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges

        left_arc = scan_data.ranges[0: 21]
        right_arc = scan_data.ranges[-20:]
        left_arc2 = scan_data.ranges[0: 35]
        right_arc2 = scan_data.ranges[-34:]
        self.front_arc2 = np.array(left_arc2[::-1] + right_arc2[::-1])

        left_arc_back = scan_data.ranges[0:160]
        right_arc_back = scan_data.ranges[-160:]
        self.minimum_value = np.array(scan_data.ranges[-170:-10]).min()
        self.minimum_vfromSmallRange = np.array(scan_data.ranges[-100:-80]).min()
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.back_arc = np.array(left_arc_back[::-1] + right_arc_back[::-1])
        # determine the minimum distance of the ranges of front_arcs:
        self.min_dis = self.front_arc.min()
        self.min_dis2 = self.front_arc2.min()
        self.min_dis_back = self.back_arc.min()
        arc_angles = np.arange(-20, 21)
        # determine the location of the angle of the min_distance:
        # in the ranges of the front of the robot:
        self.object_angle = arc_angles[np.argmin(self.front_arc)]

        self.rate.sleep()

    def find_wall(self):

       
        self.first_move = True
        t = rospy.get_rostime()
        while rospy.get_rostime().secs - t.secs < 0.5:
            continue

        while not (self.ranges):
            self.rate.sleep()

        while not self.wall_exist and not self.ctrl_c:

            # if (self.judgement(self.ranges) == States.WALL_FOLLOW):
            #     self.first_move = False
            #     self.wall_exist = True
            #     continue


            # if self.first_move:
            #     self.robot_controller.set_move_cmd(0.0, 0.9)
            #     self.robot_controller.publish()
            #     rospy.loginfo('First time spinning!!!')

            #     while (self.front_arc.min() <= 1.0) :
            
            #         continue
               
            #     self.first_move = False

            # self.robot_controller.stop()

            self.robot_controller.set_move_cmd(0.24, 0.0)
            self.robot_controller.publish()
            
            # rospy.loginfo(f'CURRENT_COLOUR----{self.current_colour}')
            while self.min_dis >= self.threshold and not self.ctrl_c:
                
                if self.current_colour != "gray" and self.current_colour != "wall":
                   
                # if (self.object_angle >= -1 and self.object_angle <= 1):
                #     rospy.loginfo('Need to turn left a bit !!!')
                #     self.leftabit()
                    continue
            rospy.loginfo('WALL detected so ADJUST direction ...')
            self.robot_controller.stop()
            self.leftabitFollowwall()
            self.wall_exist = True

           
        
            # if (self.judgement(self.ranges) == States.WALL_FOLLOW and self.ranges[-75] < 0.4  and  self.current_colour == None):
            #     self.first_move = False
            #     self.wall_exist = True
            #     continue

            # elif (self.judgement(self.ranges) != States.WALL_FOLLOW):
            #     self.leftabit()

            # self.backabit()

          
    def leftabitFollowwall(self):
        self.robot_controller.stop()
        self.robot_controller.set_move_cmd(0.0, 0.8)
        self.robot_controller.publish()
        # while self.judgement(self.ranges) != States.WALL_FOLLOW :
        #     continue
        while self.minimum_value != self.minimum_vfromSmallRange:
            continue
        self.robot_controller.stop()
       
    def obsta_avoid(self):
        if self.min_dis <= self.avoid_obstac_threshold:
            self.robot_controller.stop()
            if self.object_angle <= 0:
                rospy.loginfo('====CLOSEST object angle: LEFT==SO turn right')
                angular_z = -1.5
            else:
                rospy.loginfo('====CLOSEST object angle: RIGHT==SO turn left')
                angular_z = 1.5
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
        

            
       
    def shutdown_hook(self):
        rospy.loginfo('==== The robot STOP ====')
        self.robot_controller.stop()
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
        twosides_Threshold = 0.5
        front_Threshold = 0.4
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

        while not (self.ranges):
            self.rate.sleep()

        threeminuStart = rospy.get_rostime()

        self.find_wall()
        # while not self.ranges.any():
        #     continue

        while not self.ctrl_c and (rospy.get_rostime().secs - threeminuStart.secs < 180):
           

            state = self.judgement(self.ranges)

            if (state == States.WALL_FOLLOW):
                fr = self.ranges[-54]
                e = 0.38 - fr
                kp = 6
                # 3
                self.robot_controller.set_move_cmd(0.18, kp * e)

            elif (state == States.TOright or state == States.TURNright):
                self.robot_controller.set_move_cmd(0.18, -0.7) 

            elif (state == States.TOleft or state == States.TURNleft):
                self.robot_controller.set_move_cmd(0.18, 1.7)

            elif (state == States.STOP):
                self.robot_controller.set_move_cmd(0, 1.7)

            else:
                self.robot_controller.set_move_cmd(0, 0)

            self.adjustalittle()

            self.robot_controller.publish()
            
            self.rate.sleep()

        self.robot_controller.stop()
        rospy.loginfo('SAVE the SLAM maps')
        self.savemap()

    def backabit(self):
        rospy.loginfo('==== BACK A BIT ====')
        self.robot_controller.stop()
        self.robot_controller.set_move_cmd(-0.24, 0.0)
        self.robot_controller.publish()
        while self.min_dis <= self.adjustalittle_thre and self.min_dis_back >= 0.2:
            continue
        self.robot_controller.stop()


    def adjustalittle(self):
        self.adjustalittle_thre = 0.28
        if self.min_dis2 <= self.adjustalittle_thre:
            rospy.loginfo('==== ADJUST A BIT ====')
            self.backabit()
            self.leftabitFollowwall()




if __name__ == '__main__':
    try:
        wall_follower().main()
    except rospy.ROSInterruptException:
        pass