#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>


bool has_left = false;
bool has_right = false;

sensor_msgs::ImageConstPtr left_msg_ptr; // must be Const!
sensor_msgs::ImageConstPtr right_msg_ptr;

void leftCallback(const sensor_msgs::ImageConstPtr& msg)
{
  has_left = true;
  left_msg_ptr = msg;

  // try
  // {
  //   cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  //   cv::waitKey(10);
  // }
  // catch (cv_bridge::Exception& e)
  // {
  //   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  // }

}

void rightCallback(const sensor_msgs::ImageConstPtr& msg)
{
  has_right = true;
  right_msg_ptr = msg;
}

cv_bridge::CvImagePtr imgmsg_to_cv(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return NULL;
  }

  return cv_ptr;
}

cv_bridge::CvImagePtr depth_from_stereo(const cv_bridge::CvImagePtr& left_ptr, const cv_bridge::CvImagePtr& right_ptr)
{
  cv_bridge::CvImagePtr depth_ptr(new cv_bridge::CvImage);


  // TODO!!

  return depth_ptr;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_image_gen");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left = it.subscribe("left_image", 1, leftCallback);
  image_transport::Subscriber sub_right = it.subscribe("right_image", 1, rightCallback);
  image_transport::Publisher pub = it.advertise("depth_image", 1);  

  ros::Rate loop_rate(30);
  while (nh.ok()) {
    ros::spinOnce();
    
    if (has_left && has_right) {
      float time_diff = std::abs(left_msg_ptr->header.stamp.toSec() - right_msg_ptr->header.stamp.toSec()); 
      
      if (time_diff > 0.02) {
        ROS_ERROR("Dropping frame, timestamp diff is %.6f ", time_diff);
      }
      else {
        cv_bridge::CvImagePtr left_cv_ptr = imgmsg_to_cv(left_msg_ptr);
        cv_bridge::CvImagePtr right_cv_ptr = imgmsg_to_cv(right_msg_ptr);

        cv_bridge::CvImagePtr depth_cv_ptr = depth_from_stereo(left_cv_ptr, right_cv_ptr);

        pub.publish(depth_cv_ptr->toImageMsg());
      }
      
      has_left = false;
      has_right = false;
    }
    loop_rate.sleep();
  }

  //cv::destroyWindow("view");
}
