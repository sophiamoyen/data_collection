
import subprocess
import time
import rospy
from sensor_msgs.msg import Joy
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

def run_relay(input_topic, output_topic):
    command = ["rosrun","topic_tools","relay",input_topic,output_topic]
    subprocess.call(command)



if __name__=="__main__":

    timeout = time.time() + 10 # 10 seconds
    
    if SIMULATION == True:
        left_image = "/zed2_B/zed_node/left_raw/image_raw_color"
        right_image = "/zed2_B/zed_node/right_raw/image_raw_color"
        tiago_head = "/xtion/rgb/image_raw"


    while True:
        user_input = input('enter number: ')
        if user_input ==1:
            input_topic = left_image
        elif user_input ==2:
            input_topic = right_image
        else:
            input_topic = tiago_head

        output_topic = "/vr_relay/"

        run_relay(input_topic,output_topic)
