#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('image_folder_publisher')

import sys
import os
from os import listdir
from os.path import isfile, join

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_folder_publisher:

    def check_validity(self, img_folder):
        if img_folder == '' or not os.path.exists(img_folder) or not os.path.isdir(img_folder):
            rospy.logfatal("[%s] (image_folder) Invalid Image folder", self.__app_name)
            sys.exit(0)
        rospy.loginfo("[%s] Reading images from %s", self.__app_name, img_folder)

    def __init__(self):
        self.__app_name = "image_folder_publisher"

        self._cv_bridge = CvBridge()

        self._topic_name_left= rospy.get_param('~topic_name_left', '/image_raw')
        self._topic_name_right= rospy.get_param('~topic_name_right', '/image_raw')

        rospy.loginfo("[%s] (_topic_name_left) Publishing Images to topic  %s", self.__app_name, self._topic_name_left)
        rospy.loginfo("[%s] (topic_name_right) Publishing Images to topic  %s", self.__app_name, self._topic_name_right)

        #Declear image publisher left and right
        self._image_publisher_left = rospy.Publisher(self._topic_name_left, Image, queue_size=1)
        self._image_publisher_right = rospy.Publisher(self._topic_name_right, Image, queue_size=1)

        self._rate = rospy.get_param('~publish_rate', 1)
        rospy.loginfo("[%s] (publish_rate) Publish rate set to %s hz", self.__app_name, self._rate)

        self._sort_files = rospy.get_param('~sort_files', True)
        rospy.loginfo("[%s] (sort_files) Sort Files: %r", self.__app_name, self._sort_files)

        self._frame_id = rospy.get_param('~frame_id', 'camera')
        rospy.loginfo("[%s] (frame_id) Frame ID set to  %s", self.__app_name, self._frame_id)

        self._loop = rospy.get_param('~loop', -1)
        rospy.loginfo("[%s] (loop) Loop  %d time(s) (set it -1 for infinite)", self.__app_name, self._loop)

        self._image_folder_left = rospy.get_param('~image_folder_left', '')
        self._image_folder_right = rospy.get_param('~image_folder_right', '')

        self.check_validity(self._image_folder_left)
        self.check_validity(self._image_folder_right)

    def run(self):
        ros_rate = rospy.Rate(self._rate)

        files_in_dir_left = [f for f in listdir(self._image_folder_left) if isfile(join(self._image_folder_left, f))]
        files_in_dir_right = [f for f in listdir(self._image_folder_right) if isfile(join(self._image_folder_right, f))]
        
        if self._sort_files:
            files_in_dir_left.sort()
            files_in_dir_right.sort()
        try:
            while self._loop != 0:
                for f_left, f_right in zip(files_in_dir_left, files_in_dir_right):
                    if not rospy.is_shutdown():
                        #LEFT
                        if isfile(join(self._image_folder_left, f_left)):
                            cv_image_left = cv2.imread(join(self._image_folder_left, f_left))
                            if cv_image_left is not None:
                                ros_msg_left = self._cv_bridge.cv2_to_imgmsg(cv_image_left, "bgr8")
                                ros_msg_left.header.frame_id = self._frame_id
                                ros_msg_left.header.stamp = rospy.Time.now()
                                self._image_publisher_left.publish(ros_msg_left)
                                #rospy.loginfo("[%s] Published %s", self.__app_name, join(self._image_folder_left, f_left))
                            else:
                                rospy.loginfo("[%s] Invalid image file %s", self.__app_name, join(self._image_folder_left, f_left))

                        #RIGHT
                        if isfile(join(self._image_folder_right, f_right)):
                            cv_image_right = cv2.imread(join(self._image_folder_right, f_right))
                            if cv_image_right is not None:
                                ros_msg_right = self._cv_bridge.cv2_to_imgmsg(cv_image_right, "bgr8")
                                ros_msg_right.header.frame_id = self._frame_id
                                ros_msg_right.header.stamp = ros_msg_left.header.stamp
                                self._image_publisher_right.publish(ros_msg_right)
                                #rospy.loginfo("[%s] Published %s", self.__app_name, join(self._image_folder_right, f_right))
                            else:
                                rospy.loginfo("[%s] Invalid image file %s", self.__app_name, join(self._image_folder_right, f_right))
                        
                        ros_rate.sleep()
                    else:
                        return
                self._loop = self._loop - 1
        except CvBridgeError as e:
            rospy.logerr(e)


def main(args):
    rospy.init_node('image_folder_publisher', anonymous=True)

    image_publisher = image_folder_publisher()
    image_publisher.run()


if __name__ == '__main__':
    main(sys.argv)
