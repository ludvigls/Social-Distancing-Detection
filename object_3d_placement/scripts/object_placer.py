#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker, MarkerArray




#TODO, Initialize to None, handle None when detection has not ran yet
u, v = 120, 160 #None, None
object_position = None # [x_obj, y_obj, z_obj]
point_cloud = None

#TODO, should not be hardcoded
FOV_horizont, FOV_vertical = 90, 35 #1.5708, 0.610865 # in radians
depth_width, depth_height = None, None


def create_marker(object_position, id=0): #assumes person stands on ground (z = 0)
    #TODO, what frames are input coordinates in? Is z depth or height?? 
    marker_obj = Marker()
    marker_obj.header.frame_id = "realsense_depth_optical_frame" #eq to camera frame
    marker_obj.type = Marker.CUBE
    marker_obj.id = id

    marker_obj.pose.position.x = object_position[0]
    marker_obj.pose.position.y = object_position[1]
    marker_obj.pose.position.z = object_position[2]# + height/2

    marker_obj.pose.orientation.x = 0.0
    marker_obj.pose.orientation.y = 0.0
    marker_obj.pose.orientation.z = 0.0
    marker_obj.pose.orientation.w = 1.0
    
    marker_obj.scale.x = 1 #0.3
    marker_obj.scale.y = 1 #0.3
    marker_obj.scale.z = 1 #height

    marker_obj.color.a = 1.0
    marker_obj.color.r = 0.0
    marker_obj.color.g = 0.3
    marker_obj.color.b = 1.0

    return marker_obj

"""
import cv2
from cv_bridge import CvBridge, CvBridgeError
def imgmsg_to_cv(ros_msg):
    bridge = CvBridge()
    cv_image = None
    try:
      cv_image = bridge.imgmsg_to_cv2(ros_msg, ros_msg.encoding) #TODO, ros_msg.encoding : bgr8, 16UC1
    except CvBridgeError as e:
      print(e)

    return cv_image
"""


def convert_from_uvd(u, v, depth_image):
    # Realsense
    # K: [191.07275390625, 0.0, 160.45118713378906, 0.0, 191.07275390625, 120.63352966308594, 0.0, 0.0, 1.0]
    cx = 160.45118713378906
    cy = 120.63352966308594
    focalx = 191.07275390625
    focaly = 191.07275390625

    x_over_z = (cx - u) / focalx
    y_over_z = (cy - v) / focaly

    d = find_depth_at(depth_image, u, v)
    z =  d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
    x = x_over_z * z
    y = y_over_z * z
    return x/1000, y/1000, z/1000

def from_depth_to_pc(depth_image):
    cx = 160.45118713378906
    cy = 120.63352966308594
    focalx = 191.07275390625
    focaly = 191.07275390625

    x_over_z = (cx - u) / focalx
    y_over_z = (cy - v) / focaly

    height = depth_image.height
    width = depth_image.width
    depth_np = np.reshape(np.fromstring(depth_image.data, np.uint16), (height, width))

    z_np =  depth_np / np.sqrt(1. + x_over_z**2 + y_over_z**2)
    x_np = x_over_z * z_np
    y_np = y_over_z * z_np
    return x_np/1000, y_np/1000, z_np/1000



def find_depth_at(depth_image, u,v):
    height = depth_image.height
    width = depth_image.width
    depth_np = np.reshape(np.fromstring(depth_image.data, np.uint16), (height, width))
    return depth_np[u,v]


def find_angles(u_tilde, v_tilde, width, height, FOV_horizont, FOV_vertical):
    #Based on non linear regression
    """
    length_nounit_v = v_tilde/width
    length_nounit_u = u_tilde/height

    deg_nounit_u = -0.0504*length_nounit_u**2 + 0.4878*length_nounit_u - 0.0101
    deg_nounit_v = -0.0504*length_nounit_v**2 + 0.4878*length_nounit_v - 0.0101
    
    deg_v = deg_nounit_v*FOV_horizont
    deg_u = deg_nounit_u*FOV_vertical
    """

    #TODO, NB! This Assumes a linear relationship which may not be the case
    print("normalized", v_tilde, u_tilde)
    deg_v = FOV_horizont * v_tilde/width #Horizontal
    deg_u = FOV_vertical * u_tilde/height #Vertical

    return deg_u, deg_v

def find_dist_obj_camera(u,v, depth_image, FOV_horizont, FOV_vertical):
    w = depth_image.width
    h = depth_image.height

    #TODO, we are dealing with a left hand coordinate system (u,v,z) --> (x,y,z), Is this okay?
    #normalize image frame coordinate system, such that middle of image is origo
    u_tilde = (u-h/2) # x, left hand coord
    v_tilde = (v-w/2) # y, right hand coord

    z_obj = find_depth_at(depth_image, u,v)
    deg_u, deg_v = find_angles(u_tilde, v_tilde, w, h, FOV_horizont, FOV_vertical)
    
    # Assumes z_obj from depth image is the euclidian distance between camera and object
    x_obj = z_obj*np.sin(np.radians(deg_u))
    y_obj = z_obj*np.sin(np.radians(deg_v))

    return ( y_obj/1000, x_obj/1000, z_obj/1000) #convert from mm to meters


#ROS Callback functions
def depth_callback(data):
    global object_position, point_cloud
    object_position = find_dist_obj_camera(u,v, data, FOV_horizont, FOV_vertical)
    #object_position = convert_from_uvd(u, v, data)
    point_cloud = from_depth_to_pc(data)

def object_callback(data):
    global u,v
    u = data.x
    v = data.y

def object_placer():
    rospy.Subscriber("depth_image", Image, depth_callback)
    rospy.Subscriber("object_vector", Vector3, object_callback)

    point_cloud_pub = rospy.Publisher('pointcloud', MarkerArray, queue_size=1)

    rospy.init_node('object_placer', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        if object_position != None:
            pc_msg = MarkerArray()
            marker_obj = create_marker(object_position)
            pc_msg.markers.append(marker_obj)

            point_cloud_pub.publish(pc_msg)

        """
        if point_cloud != None:
            pc_msg = MarkerArray()
            id = 0
            for i in range(120, 121): #240, 320
                for j in range(160, 161):
                    x, y, z = point_cloud[0][i][j], point_cloud[1][i][j], point_cloud[2][i][j]
                    marker_obj = create_marker([x, y, z],  id)
                    pc_msg.markers.append(marker_obj)
                    id += 1 

            point_cloud_pub.publish(pc_msg)
        """


        rate.sleep()

if __name__ == '__main__':
    try:
        object_placer()
    except rospy.ROSInterruptException:
        pass
