#include "tek5030/opencv_camera.h"

namespace tek5030
{
OpenCVCamera::OpenCVCamera(int camera_id)
    : cap_{camera_id}
{
  if (!cap_.isOpened())
  { throw std::runtime_error("Could not open camera with id " + std::to_string(camera_id)); }
}

cv::Mat OpenCVCamera::getFrame() const
{
  cv::Mat frame;
  cap_ >> frame;

  if (frame.empty())
  { throw std::runtime_error("Lost camera feed"); }

  return frame;
}

bool OpenCVCamera::operator>>(cv::Mat& frame) const
{
  frame = getFrame();
  return !frame.empty();
}

double OpenCVCamera::getFrameRate() const
{ return cap_.get(cv::CAP_PROP_FPS); }

cv::Size OpenCVCamera::getResolution() const
{
  return
  {
    static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
    static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT))
  };
}

cv::VideoCapture& OpenCVCamera::getVideoCapture()
{ return cap_; }

bool OpenCVCamera::isOpened() const
{ return cap_.isOpened(); }
}

