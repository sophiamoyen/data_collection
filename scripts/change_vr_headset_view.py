
import subprocess
import time
import rospy
from sensor_msgs.msg import Joy , Image, CameraInfo
import numpy as np
"""
The idea of this code is to redirect different camera streams for the Vive Headset VR based on VIVE controller commands
For this to work, the topic that the VR should be listening to should be:

sub_L_str = "/vr_view/left/image"
sub_R_str = "/vr_view/right/image"
sub_i_L_str = "/vr_view/left/camera_info"
sub_i_R_str = "/vr_view/right/camera_info"

(This need to be set in the vive_node.cpp file in the windows PC)

The scripts assumes there are 3 camera views from which to switch from (one on the left of Tiago, on Tiago's head and on on right)
"""

SIMULATION = False
VIVE_CONTROLLER = 1 # 1 for setting the change vr view using left controller, 0 for right controller


class ChangeView():
    def __init__(self):

        # Getting button information from left/right vive controller
        if VIVE_CONTROLLER == 1:
            self.button_sub = rospy.Subscriber('/vive/my_left_controller_1/joy', Joy, self.button_cb)
            self.button_state = Joy()

        elif VIVE_CONTROLLER == 0:
            self.button_sub = rospy.Subscriber('/vive/my_right_controller_1/joy', Joy, self.button_cb)
            self.button_state = Joy()

        # Getting image topics according to simulation/real world
        if SIMULATION == True:
            # Left camera (ZED2 A) - as was launched in the world file
            self.zedA_left_sub = rospy.Subscriber('/zed2_A/zed_node/left_raw/image_raw_color', Image,self.zedA_left_cb)
            self.zedA_left = Image()
            self.zedA_right_sub = rospy.Subscriber('/zed2_A/zed_node/right_raw/image_raw_color', Image,self.zedA_right_cb)
            self.zedA_right = Image()
            self.zedA_left_info_sub = rospy.Subscriber('/zed2_A/zed_node/left_raw/camera_info', CameraInfo,self.zedA_left_info_cb)
            self.zedA_left_info = CameraInfo()
            self.zedA_right_info_sub = rospy.Subscriber('/zed2_A/zed_node/right_raw/camera_info', CameraInfo,self.zedA_right_info_cb)
            self.zedA_right_info = CameraInfo()

            # Right Camera (ZED2 B) - as was launched in the world file
            self.zedB_left_sub = rospy.Subscriber('/zed2_B/zed_node/left_raw/image_raw_color', Image,self.zedB_left_cb)
            self.zedB_left = Image()
            self.zedB_right_sub = rospy.Subscriber('/zed2_B/zed_node/right_raw/image_raw_color', Image,self.zedB_right_cb)
            self.zedB_right = Image()
            self.zedB_left_info_sub = rospy.Subscriber('/zed2_B/zed_node/left_raw/camera_info', CameraInfo,self.zedB_left_info_cb)
            self.zedB_left_info = CameraInfo()
            self.zedB_right_info_sub = rospy.Subscriber('/zed2_B/zed_node/right_raw/camera_info', CameraInfo,self.zedB_right_info_cb)
            self.zedB_right_info = CameraInfo()

            # Head Camera - Getting the xtion currently in simulation, but will leave the same variable names as in real world
            self.zedC_left_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image,self.zedC_left_cb)
            self.zedC_left = Image()
            self.zedC_right_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image,self.zedC_right_cb)
            self.zedC_right = Image()
            self.zedC_left_info_sub = rospy.Subscriber('/xtion/rgb/camera_info', CameraInfo,self.zedC_left_info_cb)
            self.zedC_left_info = CameraInfo()
            self.zedC_right_info_sub = rospy.Subscriber('/xtion/rgb/camera_info', CameraInfo,self.zedC_right_info_cb)
            self.zedC_right_info = CameraInfo()

        else:
            # Left camera (ZED2 A) 
            self.zedA_left_sub = rospy.Subscriber('/zedA/zed_node_A/left/image_rect_color', Image,self.zedA_left_cb)
            self.zedA_left = Image()
            self.zedA_right_sub = rospy.Subscriber('/zedA/zed_node_A/right/image_rect_color', Image,self.zedA_right_cb)
            self.zedA_right = Image()
            self.zedA_left_info_sub = rospy.Subscriber('/zedA/zed_node_A/left/camera_info', CameraInfo,self.zedA_left_info_cb)
            self.zedA_left_info = CameraInfo()
            self.zedA_right_info_sub = rospy.Subscriber('/zedA/zed_node_A/right/camera_info', CameraInfo,self.zedA_right_info_cb)
            self.zedA_right_info = CameraInfo()

            # Right Camera (ZED2 B) 
            self.zedB_left_sub = rospy.Subscriber('/zedB/zed_node_B/left/image_rect_color', Image,self.zedB_left_cb)
            self.zedB_left = Image()
            self.zedB_right_sub = rospy.Subscriber('/zedB/zed_node_B/right/image_rect_color', Image,self.zedB_right_cb)
            self.zedB_right = Image()
            self.zedB_left_info_sub = rospy.Subscriber('/zedB/zed_node_B/left/camera_info', CameraInfo,self.zedB_left_info_cb)
            self.zedB_left_info = CameraInfo()
            self.zedB_right_info_sub = rospy.Subscriber('/zedB/zed_node_B/right/camera_info', CameraInfo,self.zedB_right_info_cb)
            self.zedB_right_info = CameraInfo()

            # Head camera
            self.zedB_left_sub = rospy.Subscriber('/zedC/zed_node_C/left/image_rect_color', Image,self.zedB_left_cb)
            self.zedB_left = Image()
            self.zedB_right_sub = rospy.Subscriber('/zedC/zed_node_C/right/image_rect_color', Image,self.zedB_right_cb)
            self.zedB_right = Image()
            self.zedB_left_info_sub = rospy.Subscriber('/zedC/zed_node_C/left/camera_info', CameraInfo,self.zedB_left_info_cb)
            self.zedB_left_info = CameraInfo()
            self.zedB_right_info_sub = rospy.Subscriber('/zedC/zed_node_C/right/camera_info', CameraInfo,self.zedB_right_info_cb)
            self.zedB_right_info = CameraInfo()

        # Publishing to topics in VR
        self.view_pub_left = rospy.Publisher('/zed2/zed_node/left/image_rect_color/',Image, queue_size=5)
        self.view_pub_right = rospy.Publisher('/zed2/zed_node/right/image_rect_color/',Image, queue_size=5)
        self.view_pub_left_info = rospy.Publisher('/zed2/zed_node/left/camera_info/',CameraInfo, queue_size=5)
        self.view_pub_right_info = rospy.Publisher('/zed2/zed_node/right/camera_info/',CameraInfo, queue_size=5)

        # Initial configuration set for head camera
        self.camera_state = 1 # 0 Head, 1 Right, 2 Left

        # Mapping vr controller mousepad
        self.record_axis = [0.0 for i in range(10)]
        self.rate = rospy.Rate(200)

    def button_cb(self,button_data):  
        self.record_axis.pop(0)
        self.record_axis.append(button_data.axes[0])

        if abs(min(self.record_axis)-max(self.record_axis))>1.5:
            rospy.loginfo('Camera Changed')
            self.record_axis = [0.0 for i in range(10)]

            if self.camera_state == 0:
                self.camera_state = 1
                rospy.loginfo('View: Right')

            elif self.camera_state == 1:
                self.camera_state = 2
                rospy.loginfo('View: Left')

            elif self.camera_state == 2:
                self.camera_state = 0
                rospy.loginfo('View: Head')
           

    def zedA_left_cb(self,camera_data):
        self.zedA_left = camera_data
        if self.camera_state == 2:
            self.view_pub_left.publish(self.zedA_left)
            
    def zedA_right_cb(self,camera_data):
        self.zedA_right = camera_data
        if self.camera_state == 2:
            self.view_pub_right.publish(self.zedA_right)
            
    def zedA_left_info_cb(self,camera_data):
        self.zedA_left_info = camera_data
        if self.camera_state == 2:
            self.view_pub_left_info.publish(self.zedA_left_info)
            
    def zedA_right_info_cb(self,camera_data):
        self.zedA_right_info = camera_data
        if self.camera_state == 2:
            self.view_pub_right_info.publish(self.zedA_right_info)
            
    def zedB_left_cb(self,camera_data):
        self.zedB_left = camera_data
        if self.camera_state == 1:
            self.view_pub_left.publish(self.zedB_left)
            
    def zedB_right_cb(self,camera_data):
        self.zedB_right = camera_data
        if self.camera_state == 1:
            self.view_pub_right.publish(self.zedB_right)
            
    def zedB_left_info_cb(self,camera_data):
        self.zedB_left_info = camera_data
        if self.camera_state == 1:
            self.view_pub_left_info.publish(self.zedB_left_info)
            
    def zedB_right_info_cb(self,camera_data):
        self.zedB_right_info = camera_data
        if self.camera_state == 1:
            self.view_pub_right_info.publish(self.zedB_right_info)
            
    def zedC_left_cb(self,camera_data):
        self.zedC_left = camera_data
        if self.camera_state == 0:
            self.view_pub_left.publish(self.zedC_left)
            
    def zedC_right_cb(self,camera_data):
        self.zedC_right = camera_data
        if self.camera_state == 0:
            self.view_pub_right.publish(self.zedC_right)
            
    def zedC_left_info_cb(self,camera_data):
        self.zedC_left_info = camera_data
        if self.camera_state == 0:
            self.view_pub_left_info.publish(self.zedC_left_info)
            
    def zedC_right_info_cb(self,camera_data):
        self.zedC_right_info = camera_data
        if self.camera_state == 0:
            self.view_pub_right_info.publish(self.zedC_right_info)



if __name__=="__main__":
    rospy.init_node('camera_view_changer')
    changeview = ChangeView()
    while not rospy.is_shutdown():
        changeview.rate.sleep()
        
