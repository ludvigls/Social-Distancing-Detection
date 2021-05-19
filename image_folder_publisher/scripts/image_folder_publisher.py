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

from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class image_folder_publisher:

    def check_validity(self, img_folder):
        if img_folder == '' or not os.path.exists(img_folder) or not os.path.isdir(img_folder):
            rospy.logfatal("[%s] (image_folder) Invalid Image folder", self.__app_name)
            sys.exit(0)
        rospy.loginfo("[%s] Reading images from %s", self.__app_name, img_folder)

    def __init__(self):
        self.__app_name = "stereo_image_folder_publisher"

        self._cv_bridge = CvBridge()

        self._topic_name_left= rospy.get_param('~topic_name_left', '/image_raw')
        self._topic_name_right= rospy.get_param('~topic_name_right', '/image_raw')
        self._topic_name_depth= rospy.get_param('~topic_name_depth', '/image_raw')

        rospy.loginfo("[%s] (topic_name_left) Publishing Images to topic  %s", self.__app_name, self._topic_name_left)
        rospy.loginfo("[%s] (topic_name_right) Publishing Images to topic  %s", self.__app_name, self._topic_name_right)
        rospy.loginfo("[%s] (topic_name_depth) Publishing Images to topic  %s", self.__app_name, self._topic_name_depth)

        #Declear image publisher left and right
        self._image_publisher_left = rospy.Publisher(self._topic_name_left, Image, queue_size=1)
        self._image_publisher_right = rospy.Publisher(self._topic_name_right, Image, queue_size=1)
        self._image_publisher_depth = rospy.Publisher(self._topic_name_depth, Image, queue_size=1)

        self._info_publisher_left=rospy.Publisher('stereo_camera/left/camera_info',CameraInfo,queue_size=1)
        self._info_publisher_right=rospy.Publisher('stereo_camera/right/camera_info',CameraInfo,queue_size=1)
        self._info_publisher_depth=rospy.Publisher('stereo_camera/depth/camera_info',CameraInfo,queue_size=1)

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


                                ros_msg_depth = Image()

                                # TODO, Compute depth image from stereo...
                                # See code from lab7-maskinsyn

                                self._image_publisher_depth.publish(ros_msg_depth)

                                info=CameraInfo()
                                info.header=ros_msg_left.header
                                info.distortion_model='plumb_bob'
                                info.height=480
                                info.width=640
                                info.K=[499.9227269272732,0,319.5850778985193,0,499.6147788074767,247.3568676189872,0,0,1]
                                info.D=[-0.228883043479116,0.118796892493508,0, 0.000000000000000,0.000000000000000]
                                info.R=[1,0,0,0,1,0,0,0,1]
                                self._info_publisher_left.publish(info)


                                #info_depth = CameraInfo()
                                #TODO, add calibration params to info_depth and publish onto topic

                                self._info_publisher_depth.publish(info) #TODO, info_depth should be published instead!

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
                                info=CameraInfo()
                                info.header=ros_msg_right.header
                                info.distortion_model='plumb_bob'
                                info.height=480
                                info.width=640
                                info.K=[499.6548110675932,0,307.8663666109500,0,499.2777002132757,233.8908754867127,0,0,1]
                                info.D=[-0.233667402919539,0.132068238932958 ,0,0.000000000000000,0.000000000000000] # TODO, inserted 0 in middle, probably wrong
                                info.R=[ 0.994722424575135   ,      0.008271026929669    ,    0.102268705677217,
                                        -0.008805703908382   ,      0.999949814849582    ,     0.004777798757326,
                                        -0.102224056004653   ,     -0.005653131505306    ,    0.994745336494794]
                                self._info_publisher_right.publish(info)

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
