#!/usr/bin/env python3

# ROS libraries
import roslib
import rospy
import csv
import os
import cv2
import numpy as np
from scipy.ndimage import filters
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Bool, Int32, Float64
from geometry_msgs.msg import Twist, Point
#variables
Id_number = 0
Info_gathering_mode = True
Reached = False
Current_marker = 0
Current_angle = 0.0
CameraCenter = Point()
MarkerCenter = Point()
marker_id_list = []
marker_centers_dict = {}


def controller_callback(msg):
    global Info_gathering_mode, marker_id_list, Current_marker, CameraCenter, MarkerCenter
    global Current_angle, Reached

    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    vel = Float64()

    if Info_gathering_mode:
        if len(marker_id_list) < 7:
            Current_angle += 0.06
            vel.data = Current_angle

            if Id_number and Id_number not in marker_id_list:
                marker_id_list.append(Id_number)
                marker_id_list.sort()
        else:
            Info_gathering_mode = False
    else:
        if marker_id_list:
            Current_marker = marker_id_list[0]
            target_x = MarkerCenter.x

            if abs(CameraCenter.x - target_x) < 20 and Id_number == Current_marker:
                Reached = True
                Current_angle += 0
                vel.data = Current_angle

                cv2.circle(image_np, (int(target_x), int(MarkerCenter.y)), 20, (0, 255, 0), 3)

                Image_msg = CompressedImage()
                Image_msg.header.stamp = rospy.Time.now()
                Image_msg.format = "jpeg"
                Image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()

                image_pub.publish(Image_msg)

            if Reached:
                marker_id_list.pop(0)
                Reached = False
                Current_angle += 0
                vel.data = Current_angle

            elif CameraCenter.x > target_x and Id_number == Current_marker:
                Current_angle += 0.02
                vel.data = Current_angle

            elif CameraCenter.x < target_x and Id_number == Current_marker:
                Current_angle -= 0.02
                vel.data = Current_angle

            else:
                Current_angle += 0.05
                vel.data = Current_angle

        else:
            Current_angle += 0
            vel.data = Current_angle

    camera_holder_publisher.publish(vel)
# Publishers
image_pub = None
camera_holder_publisher = None

def rotating_camera():
    global image_pub, camera_holder_publisher

    rospy.init_node('rotating_camera')
    
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
    camera_holder_publisher = rospy.Publisher('/my_robot4/camera_holder_position_controller/command', Float64, queue_size=1)

    
    rospy.Subscriber("/marker/id_number", Int32, id_callback, queue_size=1)
    rospy.Subscriber("/marker/center_loc", Point, center_callback, queue_size=1)
    rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, controller_callback, queue_size=1)
    rospy.Subscriber("/robot/camera1/camera_info", CameraInfo, camera_callback, queue_size=1)
def id_callback(msg):
    global Id_number
    Id_number = msg.data
    
def camera_callback(msg):
    global CameraCenter
    CameraCenter.x = msg.height / 2
    CameraCenter.y = msg.width / 2



def center_callback(msg):
    global MarkerCenter, marker_id_list, marker_centers_dict, Id_number
    MarkerCenter.x = msg.x
    MarkerCenter.y = msg.y

    if Id_number and Id_number not in marker_id_list:
        marker_centers_dict[Id_number] = (msg.x, msg.y)



def main():
    rotating_camera()
    rospy.spin()

if __name__ == '__main__':
    main()

