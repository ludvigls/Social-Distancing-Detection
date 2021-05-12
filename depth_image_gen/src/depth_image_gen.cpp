#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


sensor_msgs::ImageConstPtr img_left; // must be Const!
sensor_msgs::ImageConstPtr img_right;
image_transport::Publisher pub;

void leftCallback(const sensor_msgs::ImageConstPtr& msg)
{

  //img_left = msg;
  //pub.publish(img_left);

  //pub.publish(msg);

  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void rightCallback(const sensor_msgs::ImageConstPtr& msg)
{
  img_right = msg;

  /*
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  */
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_image_gen");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left = it.subscribe("left_image", 1, leftCallback);
  image_transport::Subscriber sub_right = it.subscribe("right_image", 1, rightCallback);
  image_transport::Publisher pub = it.advertise("depth_image", 1);
  
  ros::spin();

  /*
  ros::Rate loop_rate(30);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  */

  cv::destroyWindow("view");
}
