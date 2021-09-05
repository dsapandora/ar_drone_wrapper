#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyleft (c) 2018 CITIC.
## This module export all the topics needed for the drone work
# /ardrone/cmd_vel
# /ardrone/nav_data
# /ardrone/takeoff
# /ardrone/land
# /ardrone/reset
# /ardrone/odom
# /ardrone/front_camera/raw_image
# /ardrone/front_camera/raw_image_compressed
# /ardrone/altitude
# /ardrone/battery
# /ardrone/tf
# /ardrone/mag
# Tag detection, I will work on it in the next version

# Authors:
#   * Ariel Vernaza (DSAPANDORA)
#     avernaza@citicup.org
import sys
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import rospy
from ar_drone_wrapper.msg import Navdata
# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2
from libardrone import ardrone
from multiprocessing import Process

drone = None
pub_nav_data = rospy.Publisher("/ardrone/navdata", Navdata, queue_size=1)
front_image_pub = rospy.Publisher("/ardrone/front_camera/raw_image_compressed", CompressedImage, queue_size=10)
#bottom_image_pub = rospy.Publisher("/ardrone/bottom_camera/raw_image_compressed", CompressedImage, queue_size=10)


def images(drone):
    rospy.loginfo("Image info thread")
    if drone:
       # drone.set_camera_view(False)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        compressed_images = cv2.imencode('.jpg', drone.get_image())
        msg.data = np.array(compressed_images[1]).tostring()
        front_image_pub.publish(msg)
       # drone.set_camera_view(True)
       # msg = CompressedImage()
      #  msg.header.stamp = rospy.Time.now()
      #  msg.format = "jpeg"
      #  compressed_images = cv2.imencode('.jpg', drone.get_image())
      #  msg.data = np.array(compressed_images[1]).tostring()
      #  bottom_image_pub.publish(msg)
    else:
        rospy.loginfo("No inicio")

def nav_data(drone):
    rospy.loginfo("nav data thread")
    data = Navdata()
    if drone:
        nav_data_drone = drone.get_navdata()
        data.batteryPercent = nav_data_drone[0]["battery"]
        data.state = nav_data_drone[0]["ctrl_state"]
        data.vx = nav_data_drone[0]["vx"]
        data.vy = nav_data_drone[0]["vy"]
        data.vz = nav_data_drone[0]["vz"]
        data.altd = nav_data_drone[0]["altitude"]
        data.rotX = nav_data_drone[0]["phi"]
        data.rotY = nav_data_drone[0]["theta"]
        data.rotZ = nav_data_drone[0]["psi"]
        pub_nav_data.publish(data)
    else:
        rospy.loginfo("No data to publish")

def cmd_vel(move_data):
    global drone
    if move_data.linear.x == -1:
        drone.move_right()
    if move_data.linear.y == 1:
        drone.move_forward()
    if move_data.linear.z == 1:
        drone.move_up()
    if move_data.linear.x == 1:
        drone.move_left()
    if move_data.linear.y == -1:
        drone.move_backward()
    if move_data.linear.z == -1:
        drone.move_down()
    images(drone)
    #nav_data(drone)

def takeoff(_data):
    global drone
    drone.takeoff()
    drone.hover()

def land(_data):
    global drone
    drone.land()

def reset(_data):
    global drone
    drone.reset()

def main(args):
    global drone
    rospy.init_node('drone_driver', anonymous=True)
    rospy.loginfo("Starting drone connection")
    drone = ardrone.ARDrone(True)
    rospy.loginfo("Connection Success!!")
    drone.reset()
    cmd_vel_sub = rospy.Subscriber("/ardrone/cmd_vel", Twist, cmd_vel,  queue_size = 1)
    takeoff_sub = rospy.Subscriber("/ardrone/takeoff", Empty, takeoff,  queue_size = 1)
    land_sub = rospy.Subscriber("/ardrone/land", Empty, land,  queue_size = 1)
    reset_sub = rospy.Subscriber("/ardrone/reset", Empty, reset,  queue_size = 1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        drone.halt()
        nav_data_thread.terminate()
        print ("Shutting down ROS ARDRONE DRIVER module")

if __name__ == '__main__':
    main(sys.argv)
