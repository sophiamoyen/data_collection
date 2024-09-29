
import subprocess
import time
import rospy
from sensor_msgs.msg import Joy , Image
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

SIMULATION = True
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
            # Left camera
            self.left_camera_left_sub = rospy.Subscriber('/zed2_A/zed_node/left_raw/image_raw_color', Image,self.left_camera_left_cb)
            self.left_camera_left = Image()
            self.left_camera_sub_right = rospy.Subscriber('/zed2_A/zed_node/right_raw/image_raw_color', Image,self.left_camera_right_cb)
            self.left_camera_right = Image()

            # Right Camera
            self.right_camera_left_sub = rospy.Subscriber('/zed2_B/zed_node/left_raw/image_raw_color', Image,self.right_camera_left_cb)
            self.right_camera_left = Image()
            self.right_camera_right_sub = rospy.Subscriber('/zed2_B/zed_node/left_raw/image_raw_color', Image,self.right_camera_right_cb)
            self.right_camera_right = Image()

            # Head Camera
            self.head_camera = rospy.Subscriber('/xtion/rgb/image_rect_color', Image,self.head_camera_cb)
            self.head_camera = Image()

        else:
            a = 0

        # Publishing to topics in VR
        self.view_pub_left = rospy.Publisher('/vr_view/left/image/',Image, queue_size=5)
        self.view_pub_right = rospy.Publisher('/vr_view/right/image/',Image, queue_size=5)
        self.view_pub_left_info = rospy.Publisher('/vr_view/left/camera_info/',Image, queue_size=5)
        self.view_pub_right_info = rospy.Publisher('/vr_view/right/camera_info/',Image, queue_size=5)

        # Initial configuration set for head camera
        self.camera_state = 1 # 0 Head, 1 Right, 2 Left

        #self.old_button_state = 0

        self.record_axis = [0.0 for i in range(10)]
        self.rate = rospy.Rate(200)

    def button_cb(self,button_data):  
        '''
        self.button_state = button_data
        if self.old_button_state == 0 and self.button_state.buttons[0] == 1:
            rospy.loginfo('Camera Changed')
            if self.camera_state == 0:
                self.camera_state = 1
                rospy.loginfo('View: Left')
            else:
                self.camera_state = 0
                rospy.loginfo('View: Right')
        self.old_button_state = self.button_state.buttons[0]
                '''
        self.record_axis.pop(0)
        self.record_axis.append(button_data.axes[0])
        #rospy.loginfo(self.record_axis)
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
           

    def left_camera_left_cb(self,camera_data):
        self.left_camera_left = camera_data
        if self.camera_state:
            self.view_pub.publish(self.left_camera_left)

    def right_camera_cb(self,camera_data):
        self.right_camera = camera_data
        if not self.camera_state:
            self.view_pub.publish(self.right_camera)






if __name__=="__main__":
    rospy.init_node('camera_view_changer')
    changeview = ChangeView()
    while not rospy.is_shutdown():
        changeview.rate.sleep()
        
