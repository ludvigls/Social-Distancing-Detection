#include "tek5030/dual_camera.h"
#include <iomanip>
#include <ostream>



namespace tek5030
{
DualCamera::DualCamera(int left_camera_id, int right_camera_id)
    : left_cap_{left_camera_id}
    , right_cap_{right_camera_id}
{
  if (!left_cap_.isOpened())
  { throw std::runtime_error("Could not open left camera (id " + std::to_string(left_camera_id) + ")"); }

  if (!right_cap_.isOpened())
  { throw std::runtime_error("Could not open right camera (id " + std::to_string(right_camera_id) + ")"); }
}

StereoPair DualCamera::getStereoPair() const
{
  // Use VideoCapture::grab() to make sure that the images are captured as near in time as possible.
  left_cap_.grab();
  right_cap_.grab();

  StereoPair stereo_pair{};
  // The captured images are retrieved with VideoCapture::retrieve().
  const bool left_ok = left_cap_.retrieve(stereo_pair.left);
  const bool right_ok = right_cap_.retrieve(stereo_pair.right);

  if (!left_ok)
  { throw std::runtime_error{"Could not capture from left camera"}; }

  if (!right_ok)
  { throw std::runtime_error{"Could not capture from right camera"}; }

  return stereo_pair;
}

cv::VideoCapture& DualCamera::getVideoCapture(const DualCamera::Camera& camera)
{
  return (camera == Camera::LEFT) ? left_cap_ : right_cap_;
}
}
