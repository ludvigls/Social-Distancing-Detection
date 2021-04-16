#include "opencv2/highgui.hpp"
#include "tek5030/dual_camera.h"
#include <iomanip>
#include <iostream>

using namespace tek5030;

std::string getStatus(tek5030::DualCamera& camera);

int main() try
{
  DualCamera cam(0,1);

  constexpr bool sync_focus{true};

  auto& left = cam.getVideoCapture(DualCamera::Camera::LEFT);
  auto& right = cam.getVideoCapture(DualCamera::Camera::RIGHT);
  left.set(cv::CAP_PROP_AUTOFOCUS, !sync_focus);

  //std::cout << getStatus(cam) << std::endl;

  const std::string window_name{"frame"};
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);

#define ever (;;)
  for ever
  {
    if (sync_focus)
    { left.set(cv::CAP_PROP_FOCUS, std::min(1., right.get(cv::CAP_PROP_FOCUS) + .05)); }

    const auto pair = cam.getStereoPair().hconcat();

    cv::imshow(window_name, pair);

    if (cv::waitKey(1) >= 0)
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

std::string getStatus(tek5030::DualCamera& camera)
{
  const auto& left_cap = camera.getVideoCapture(DualCamera::Camera::LEFT);
  const auto& right_cap = camera.getVideoCapture(DualCamera::Camera::RIGHT);
  constexpr int width{7};
  std::ostringstream os;
  os << std::boolalpha;
  os
      << "camera    |"
      << std::setw(width) << "left" <<  " |"
      << std::setw(width) << "right" << " |"
      << "\nautofocus |"
      << std::setw(width) << (bool(left_cap.get(cv::CAP_PROP_AUTOFOCUS)) ? "true" : "follow") <<  " |"
      << std::setw(width) << bool(right_cap.get(cv::CAP_PROP_AUTOFOCUS)) << " |"
      << "\nfocus     |"
      << std::setw(width) << left_cap.get(cv::CAP_PROP_FOCUS) <<  " |"
      << std::setw(width) << right_cap.get(cv::CAP_PROP_FOCUS) << " |"
      ;

  return os.str();
}
