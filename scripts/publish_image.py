#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('picam_ros')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import picamera
import numpy as np

class ImagePublisher:

  def __init__(self):
    self.image_pub = rospy.Publisher("picamera/image",Image)

    self.bridge = CvBridge()

  def publish_img(self,cv_image):
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  imgpub = ImagePublisher()
  rospy.init_node('image_publisher', anonymous=True)
  rate = rospy.Rate(15)
  with picamera.PiCamera() as camera:
    camera.sensor_mode = 2 #2592x1944
    camera.framerate = 15
    time.sleep(2)
    image = np.empty((2594, 1944, 3), dtype=np.uint8)
    while not rospy.is_shutdown():
      camera.capture(image, 'bgr')
      imgpub.publish_img(image)
      rate.sleep()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  try:
    main(sys.argv)
  except rospy.ROSInterruptException:
    pass
