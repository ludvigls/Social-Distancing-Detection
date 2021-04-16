#pragma once

#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"

namespace tek5030
{
/// \brief Interact with a cv::VideoCapture through a even more simplified interface.
class OpenCVCamera
{
public:
  /// \brief Create a new instance of a OpenCVCamera.
  /// \param camera_id The device id of the camera.
  explicit OpenCVCamera(int camera_id);

  bool isOpened() const;

  /// \return A new frame.
  cv::Mat getFrame() const;

  /// \brief Stream operator returning a new frame.
  /// \param frame The cv::Mat in which to store the frame.
  bool operator>>(cv::Mat& frame) const;

  double getFrameRate() const;

  cv::Size getResolution() const;

  cv::VideoCapture& getVideoCapture();

private:
  mutable cv::VideoCapture cap_;
};
}
