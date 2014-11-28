#!/usr/bin/python

import rospy
import tf
from tf.transformations import *
import math

import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# Position correction
POS_CORRECT = (
  0.0,
  -0.015,
  0.0)

# Angle adjustment, empirically measured
ORI_CORRECT = (
  0.10839177755163054,
  0.004772270652256854,
  0.027583502764761555,
  0.9937140425531109)

if __name__ == '__main__':
  rospy.init_node('headcam_corrector')
  br = tf.TransformBroadcaster()
  r = rospy.Rate(10)
 
  bridge = CvBridge()
  cam_pub = rospy.Publisher('/cameras/corrected_head/image', Image)
  info_pub = rospy.Publisher('/cameras/corrected_head/camera_info', CameraInfo)

  # Publish camera info relative to corrected frame
  def correct_camera_info(message):
    message.header.frame_id = '/corrected_head'
    info_pub.publish(message)

  # Flip image around x and y axes and republish relative to corrected_head frame
  def correct_image(message):
    try:
      img = bridge.imgmsg_to_cv2(message, "bgr8")
      msg = bridge.cv2_to_imgmsg(cv2.flip(img,-1),"bgr8")
      msg.header = message.header
      msg.header.frame_id = '/corrected_head'
      cam_pub.publish(msg)
    except CvBridgeError, e:
      print e

  rospy.Subscriber('/cameras/head_camera/image', Image, correct_image)
  rospy.Subscriber('/cameras/head_camera/camera_info', CameraInfo, correct_camera_info)

  # Publish tf data for corrected_head frame
  while not rospy.is_shutdown():
    br.sendTransform(POS_CORRECT, ORI_CORRECT,
      rospy.Time().now(),'corrected_head','head_camera')
    r.sleep()
