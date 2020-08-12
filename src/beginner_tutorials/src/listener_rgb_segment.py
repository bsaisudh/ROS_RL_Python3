#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, bsaisudh
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of bsaisudh. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

# Simple talker demo that listens to RGB images in sensor_msgs.msg/Image published
# to the '/camera/rgb/image_raw' topic and publish to 'test_image_topic_rgb' topic


import sys
import cv2
import torch

import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


print("Python version")
print(sys.version)
print("Version info.")
print(sys.version_info)
print("Pytorch details")
print("Version : ", torch.__version__)
print("CUDA availability : ", torch.cuda.is_available())

class image_converter:

    def __init__(self):
        self.image_pub_rgb = rospy.Publisher("test_image_topic_rgb", Image)

        self.bridge = CvBridge()
        self.image_sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_rgb)

    def callback_rgb(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            (rows, cols, channels) = cv_image.shape
            rospy.loginfo(rospy.get_caller_id() + ' Recieved RGB image of size : %d, %d, %d', rows, cols, channels)
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (150, 150), 50, (255, 0, 0), -1 )

        cv2.imshow("Image window RGB", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub_rgb.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    ic = image_converter()

    rospy.init_node('listener_segment', anonymous=True)

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()
