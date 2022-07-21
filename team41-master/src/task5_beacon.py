#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
from operator import truediv
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import the tb3 modules (which needs to exist within the "week6_vision" package)
from tb3 import Tb3Move

from pathlib import Path
import roslaunch

waiting_for_image = True


class colour_search():
    

    def __init__(self):
        print("\n in file -------------")
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        beacon_image = False
        target_beacon = False

        self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw",
            Image, self.camera_callback) ####USE FOR ACTUAL ROBOT
        # self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
        #     Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.base_image_path = Path.home()


        target_colours = {
            'blue': {
                # 'lower' : (115, 224, 100),  #sim
                # 'upper': (130, 255, 255), #sim
                'lower': [(100, 150, 0)],
                'upper': [(130, 255, 255)],
                'm00_min' : 200000
            }, 
            'red': {
                'lower' : [(150, 145, 25)], 
                'upper': [(190, 255, 255)], 
                 'm00_min' : 100000
            }, 
            'yellow': {
                'lower' : [(19, 135, 75)], 
                'upper': [(35, 255, 255)], 
                'm00_min' : 150000
            }, 
            'green': {
                'lower' : [(75, 125, 25)], 
                'upper': [(90, 255, 255)], 
                'm00_min' : 350000
            }, 
        }

        self.rate = rospy.Rate(5)
        self.cy =0 
        self.m00 = 0
        self.target_colour = "blue"
        self.target_colour = rospy.get_param('task5_beacon/colour')
        print("*******************")
        print(self.target_colour)
        print("*******************")
        self.lower = target_colours[self.target_colour]['lower']
        self.upper = target_colours[self.target_colour]['upper']
        self.m00_min = target_colours[self.target_colour]['m00_min']
        print("~###################################################")
        print(self.upper)
        print(self.lower)
        print("~###################################################")
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)
    

        # self.m00_min = 100000 #blue ON REAL ROBOT
        ### self.m00_min = 150000 #yellow
        ### self.m00_min = 100000 #green



        ## Thresholds for ["Blue"] ON REAL ROBOT
        # self.lower = [(-5, 180, 100)]
        # self.upper = [(5, 255, 255)]

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True


    def show_and_save_image(self, img, img_name):
        print("\n IN FUNC ---------------------")
        base_image_path = base_image_path = Path.home()
        base_image_path.mkdir(parents=True, exist_ok=True)

        full_image_path = base_image_path.joinpath(f'catkin_ws/src/team41/snaps/{img_name}.jpg')

        cv2.imwrite(str(full_image_path), img)
        print(f"Saved an image to '{full_image_path}'\n"
            f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
            f"file size = {full_image_path.stat().st_size} bytes")
    
    def camera_callback(self, img_data):
        print("/n in callback func ---------")
        waiting_for_image = True
        print(self.cy)
        print(self.m00)

        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)


        if waiting_for_image == True:

            height, width, _ = cv_img.shape
            crop_width = width - 800
            crop_height = 400
            crop_x = int((width/2) - (crop_width/2))
            crop_y = int((height/2) - (crop_height/2))

            crop_img = cv_img[240:480, 0:800]
            hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # create a single mask to accommodate all four dectection colours:
            for i in range(1):
                if i == 0:
                    mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
                else:
                    mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

            m = cv2.moments(mask)
                
            self.m00 = m["m00"]
            self.cy = m["m10"] / (m["m00"] + 1e-5)

            if self.m00 > self.m00_min:
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        
            cv2.imshow("mask image", mask)

            cv2.imshow("cropped image", crop_img)
            cv2.waitKey(1)

            height, width, channels = cv_img.shape

            print(f"Obtained an image of height {height}px and width {width}px.")
            if (self.m00 > self.m00_min) and (self.cy >= 560-100 and self.cy <= 560+100):
                # blob detected
                print("\n found itttttt ---")
                self.show_and_save_image(cv_img ,img_name = "the_beacon")


            waiting_for_image = False
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
        global waiting_for_image 
        threeminuStart = rospy.get_rostime()

        while (rospy.get_rostime().secs - threeminuStart.secs < 180):




            
            self.robot_controller.publish()
            self.rate.sleep()
        self.savemap()
            
if __name__ == "__main__":
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass