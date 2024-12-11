#!/usr/bin/env python3
import roslib
import rospy
import cvs
import os
import sys
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2
import imutils
import numpy as np
from scipy.ndimage import filters
import time
import math
#Variables
camera_center = Point()
marker_center = Point()
marker_id_list = []
marker_centers_dict = {}
id_number = 0
info_gathering_mode = True
reached = False
current_marker = 0

# Publishers
image_pub = None
velocity_publisher = None

# Callback to find the ID number for the marker
def id_callback(msg):
    global id_number
    id_number = msg.data

# Callback to find the marker center
def center_callback(msg):
    global marker_center, id_number, marker_id_list, marker_centers_dict
    marker_center.x = msg.x
    marker_center.y = msg.y

    if id_number and id_number not in marker_id_list:
        marker_centers_dict[id_number] = (msg.x, msg.y)
# Callback to find the center of the camera
def camera_callback(msg):
    global camera_center
    camera_center.x = msg.height / 2
    camera_center.y = msg.width / 2

# Main control callback
def controller_callback(msg):
    global info_gathering_mode, marker_id_list, id_number, camera_center, marker_center, reached, current_marker

    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    vel = Twist()

    if info_gathering_mode:
        if len(marker_id_list) < 7:
            vel.linear.x = 0
            vel.angular.z = 0.7

            if id_number and id_number not in marker_id_list:
                marker_id_list.append(id_number)
                marker_id_list.sort()
        else:
            info_gathering_mode = False
    else:
        if len(marker_id_list) > 0:
            current_marker = marker_id_list[0]
            target_x = marker_center.x
            target_y = marker_center.y

            if abs(camera_center.x - target_x) < 10 and id_number == current_marker:
                reached = True
                vel.angular.z = 0

                cv2.circle(image_np, (int(target_x), int(target_y)), 20, (0, 255, 0), 3)
                image_msg = CompressedImage()
                image_msg.header.stamp = rospy.Time.now()
                image_msg.format = "jpeg"
                image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
                image_pub.publish(image_msg)

            if reached:
                marker_id_list.pop(0)
                reached = False
                vel.angular.z = 0
            elif camera_center.x > target_x and id_number == current_marker:
                vel.angular.z = 0.4
            elif camera_center.x < target_x and id_number == current_marker:
                vel.angular.z = -0.4
            else:
                vel.angular.z = 0.4
        else:
            vel.angular.z = 0

    velocity_publisher.publish(vel)

# Initialize publishers and subscribers
def setup_publishers_and_subscribers():
    global image_pub, velocity_publisher
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, controller_callback, queue_size=1)
    rospy.Subscriber("/robot/camera1/camera_info", CameraInfo, camera_callback, queue_size=1)
    rospy.Subscriber("/marker/id_number", Int32, id_callback, queue_size=1)
    rospy.Subscriber("/marker/center_loc", Point, center_callback, queue_size=1)

# Main function
def main():
    rospy.init_node('fixed_camera')
    setup_publishers_and_subscribers()
    rospy.spin()

if __name__ == '__main__':
    main()

