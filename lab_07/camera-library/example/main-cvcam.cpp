#include "opencv2/highgui.hpp"
#include "tek5030/opencv_camera.h"
#include <iostream>

int main() try
{
  using namespace tek5030;

  std::cout << "Hello, World!" << std::endl;

  OpenCVCamera cam(0);
  const std::string window_name{"frame"};
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);

#define ever (;;)
  for ever
  {
    cv::Mat frame;
    cam >> frame;

    cv::imshow(window_name, frame);

    if (cv::waitKey(1) >= 0)
    { break; }
  }
  return EXIT_SUCCESS;
}
catch (const std::exception& e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}