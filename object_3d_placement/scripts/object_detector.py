#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError

curr_image = None

def image_callback(data):
    print("GOT IMG")
    bridge = CvBridge()
    try:
      global curr_image
      curr_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

def object_detection(cv_image):
    obj_vec = Vector3()

    #use Birk code to detect obstacle ... 
    obj_vec.x = 0
    obj_vec.y = 0 
    obj_vec.z = 0
    #end ...
    
    return obj_vec

def object_detector():
    rospy.Subscriber("left_image", Image, image_callback)
    object_pub = rospy.Publisher('object_vector', Vector3, queue_size=1)
    rospy.init_node('object_detector', anonymous=True)
    rate = rospy.Rate(5) # 10hz

    while not rospy.is_shutdown():
        
        obj_vec = object_detection(curr_image)
        object_pub.publish(obj_vec)

        rate.sleep()

if __name__ == '__main__':
    try:
        object_detector()
    except rospy.ROSInterruptException:
        pass
