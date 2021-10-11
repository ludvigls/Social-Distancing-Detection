#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker


def create_person(x_camera,y_camera): #assumes person stands on ground (z = 0)
    marker_obj = Marker()
    marker_obj.header.frame_id = "base_link" #eq to camera frame
    marker_obj.type = Marker.CUBE

    height = 1.7 # average height of person
    marker_obj.pose.position.x = x_camera
    marker_obj.pose.position.y = y_camera
    marker_obj.pose.position.z = height/2

    marker_obj.pose.orientation.x = 0.0
    marker_obj.pose.orientation.y = 0.0
    marker_obj.pose.orientation.z = 0.0
    marker_obj.pose.orientation.w = 1.0
    
    marker_obj.scale.x = 0.3
    marker_obj.scale.y = 0.3
    marker_obj.scale.z = height

    marker_obj.color.a = 1.0
    marker_obj.color.r = 0.0
    marker_obj.color.g = 0.3
    marker_obj.color.b = 1.0

    return marker_obj

def depth_callback(data):
    print("GOT IMG")

def object_callback(data):
    print("GOT OBJ")

def object_placer():
    rospy.Subscriber("depth_image", Image, depth_callback)
    rospy.Subscriber("object_vector", Vector3, object_callback)

    marker_pub = rospy.Publisher('object_marker', Marker, queue_size=10)
    rospy.init_node('object_placer', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        #Calculations... 

        x_camera = 0
        y_camera = 0

        marker_obj = create_person(x_camera, y_camera)
        marker_pub.publish(marker_obj)

        rate.sleep()

if __name__ == '__main__':
    try:
        object_placer()
    except rospy.ROSInterruptException:
        pass
