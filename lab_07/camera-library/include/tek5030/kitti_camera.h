#pragma once

#include "stereo_pair.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"

#include <map>
#include <string>

namespace tek5030
{
/// \brief Interact with a Kitti dataset as a stereo camera.
class KittiCamera
{
public:
  struct Calibration
  {
    cv::Size size;
    cv::Mat K;
    cv::Mat d;
    cv::Mat rotation;
    cv::Mat translation;

    struct Rectified
    {
      cv::Size size;
      cv::Mat R;
      cv::Mat P;
    } rectified;
  };

  enum class Cam
  { GrayLeft, GrayRight, ColorLeft, ColorRight, };

  /// \brief Create a new instance of a DualCamera.
  /// \param left_camera_id The device id of the left camera.
  /// \param right_camera_id The device id of the right camera.
  explicit KittiCamera(
    const std::string& dataset_path,
    const std::string& calib_path,
    bool color
    );

  /// \brief Retrieve a pair of images from the camera.
  /// The images are captured as near in time as possible.
  /// \return The captured images.
  StereoPair getStereoPair() const;

  size_t getFrameCount() const;

  Calibration getCalibration(Cam cam) const;

  bool color() const;

private:
  mutable size_t frame_count_ = 0;
  bool color_;
  mutable cv::VideoCapture left_cap_;
  mutable cv::VideoCapture right_cap_;

  std::map<std::string, KittiCamera::Calibration> calibration_map_;
};
}
