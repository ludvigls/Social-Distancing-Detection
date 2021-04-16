#include "cv_stereo_matcher_wrap.h"

std::pair<int, int> getMinAndNumDisp(const cv::Mat& Q, double min_z, uint32_t divisor)
{
  const auto fu = Q.at<double>(2, 3);
  const auto bx = 1.0 / Q.at<double>(3, 2);
  const double max_disp = fu * (bx / std::max(0.1, min_z));

  const int num_disp = static_cast<int>(std::ceil(std::abs(max_disp) / divisor)) * divisor;

  int min_disp = 0;
  if (max_disp < 0)
  { min_disp = -num_disp; }

  return {min_disp, num_disp};
}

CvStereoMatcherWrap::CvStereoMatcherWrap(cv::Ptr<cv::stereo::StereoMatcher> ptr)
    : matcher_(std::move(ptr))
{}

cv::Mat CvStereoMatcherWrap::compute(const cv::Mat& rectified_left,
                                     const cv::Mat& rectified_right)
{
  if (matcher_.empty())
  { return {}; }

  cv::Mat left_padded, right_padded;

  cv::copyMakeBorder(rectified_left, left_padded, 0, 0,
                     matcher_->getNumDisparities(), 0,
                     cv::BORDER_CONSTANT, cv::Scalar::all(0));
  cv::copyMakeBorder(rectified_right, right_padded, 0, 0,
                     matcher_->getNumDisparities(), 0,
                     cv::BORDER_CONSTANT, cv::Scalar::all(0));

  cv::Mat disparity_padded;
  matcher_->compute(left_padded, right_padded, disparity_padded);

  // Unpad
  const int pixels_to_unpad_left  = matcher_->getNumDisparities();
  const int pixels_to_unpad_right = left_padded.cols;

  const cv::Mat disparity_16bit_fixed(
      disparity_padded(cv::Range::all(), cv::Range(pixels_to_unpad_left, pixels_to_unpad_right))
  );

  // Convert from 16 bit fixed point
  cv::Mat disparity;
  constexpr float ratio_1_over_16 = 1.0f / 16.0f;
  disparity_16bit_fixed.convertTo(disparity, CV_32FC1, ratio_1_over_16);

  return disparity;
}

cv::Mat CvStereoMatcherWrap::compute(const tek5030::StereoPair& rectified)
{
  return compute(rectified.left, rectified.right);
}