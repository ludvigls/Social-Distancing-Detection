#include "tek5030/kitti_camera.h"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace tek5030;

int main(int argc, char** argv) try
{
  if (argc < 3)
  {
    const std::string path{argv[0]};
    const auto basename = path.substr(path.find_last_of("/\\") + 1);
    std::cerr
      << "Usage:\n\t" << basename
      << " <path to dataset>"
      << " <path to calib>"
      << std::endl;
    return EXIT_FAILURE;
  }
  const std::string dataset_path{argv[1]};
  const std::string calib_path{argv[2]};
  const bool color = false;
  KittiCamera cam(dataset_path, calib_path, color);

  const auto calib = cam.getCalibration(KittiCamera::Cam::GrayRight);
  std::cout << calib.rotation << std::endl;
  std::cout << calib.translation << std::endl;


  const std::string window_name{"frame"};
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);

#define ever (;;)

  for ever
  {
    const auto [left, right] = cam.getStereoPair();

    cv::Mat pair;
    cv::hconcat(left, right, pair);
    cv::imshow(window_name, pair);

    if (cv::waitKey(100) == 'q')
    { break; }
  }
  std::cout  << std::endl;
  return EXIT_SUCCESS;
}
catch (const std::exception& e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
