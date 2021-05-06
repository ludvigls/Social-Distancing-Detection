#pragma once

#include "tek5030/stereo_pair.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/stereo.hpp"

/// Helper function to compute minimum - and number of disparities for a stereo algorithm.
/// Given shortest range interesting for stereo processing, calculate values for
/// minimum disparity and number of disparities.
/// \param Q matrix Q from stereo calibration (rectification)
/// \param min_z shortest interesting range (units must match baseline in Q (mm vs m))
/// \param divisor In case the number of disparities must be divisible by some number (default 16).
/// \return std::pair{min_disp, num_disp}
std::pair<int, int> getMinAndNumDisp(const cv::Mat& Q, double min_z, uint32_t divisor = 16);

/// Wrapper for cv::StereoMatcher which takes care of the padding details
class CvStereoMatcherWrap
{
public:
  /// Constructor
  /// \param ptr Pointer to cv::StereoMatcher.
  /// Settings for the matcher are controlled outside of this class.
  explicit CvStereoMatcherWrap(cv::Ptr<cv::stereo::StereoMatcher> ptr);

  /// Run the matcher's `compute` method, while also taking care of padding
  /// the input images and converting the resulting image to CV_32FC1.
  /// \param rectified_left rectified master image (view of disparity image)
  /// \param rectified_right  rectified slave image
  /// \return disparity image with values {0,1,...,numDisp}, invalid pixels marked -1.
  cv::Mat compute(const cv::Mat& rectified_left, const cv::Mat& rectified_right);

  /// This is an overloaded member function, provided for convenience.
  /// It differs from the above function only in what argument(s) it accepts.
  /// \param rectified pair
  cv::Mat compute(const tek5030::StereoPair& rectified);

private:
  cv::Ptr<cv::stereo::StereoMatcher> matcher_;
};
