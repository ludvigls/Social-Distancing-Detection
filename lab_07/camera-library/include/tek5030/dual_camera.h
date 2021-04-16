#pragma once

#include "stereo_pair.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"

namespace tek5030
{
/// \brief Interact with two connected cameras as a stereo camera.
class DualCamera
{
public:
  enum class Camera {LEFT, RIGHT};

  /// \brief Create a new instance of a DualCamera.
  /// \param left_camera_id The device id of the left camera.
  /// \param right_camera_id The device id of the right camera.
  explicit DualCamera(int left_camera_id, int right_camera_id);

  /// \brief Retrieve a pair of images from the camera.
  /// The images are captured as near in time as possible.
  /// \return The captured images.
  StereoPair getStereoPair() const;

  cv::VideoCapture& getVideoCapture(const Camera&);

private:
  mutable cv::VideoCapture left_cap_;
  mutable cv::VideoCapture right_cap_;
};
}
