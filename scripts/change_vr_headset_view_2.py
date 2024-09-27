
import subprocess
import time
import rospy
from sensor_msgs.msg import Joy , Image
import numpy as np
"""
The idea of this code is to redirect different camera streams for the Vive Headset VR based on VIVE controller commands
For this to work, the topic that the VR should be listening to should be:

sub_L_str = "/vr_view_left"
sub_R_str = "/vr_view_right"
sub_i_L_str = "/vr_view_left_camera_info"
sub_i_R_str = "/vr_view_right_camera_info"

This need to be set in the vive_node.cpp file in the windows PC witht 
"""

SIMULATION = True


class ChangeView():
    def __init__(self):
        
        self.button_sub = rospy.Subscriber('/vive/my_left_controller_1/joy', Joy, self.button_cb)
        self.button_state = Joy()

        self.left_camera_sub = rospy.Subscriber('/zed2_B/zed_node/left_raw/image_raw_color', Image,self.left_camera_cb)
        self.left_camera = Image()

        self.right_camera_sub = rospy.Subscriber('/zed2_A/zed_node/left_raw/image_raw_color', Image,self.right_camera_cb)
        self.right_camera = Image()

        self.view_pub = rospy.Publisher('/vr_relay/',Image, queue_size=5)

        self.camera_state = 1 # 0 Right 1 Left
        self.old_button_state = 0

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
                rospy.loginfo('View: Left')
            else:
                self.camera_state = 0
                rospy.loginfo('View: Right')
           
        

    def left_camera_cb(self,camera_data):
        self.left_camera = camera_data
        if self.camera_state:
            self.view_pub.publish(self.left_camera)

    def right_camera_cb(self,camera_data):
        self.right_camera = camera_data
        if not self.camera_state:
            self.view_pub.publish(self.right_camera)






if __name__=="__main__":
    rospy.init_node('camera_view_changer')
    changeview = ChangeView()
    while not rospy.is_shutdown():
        changeview.rate.sleep()
        
