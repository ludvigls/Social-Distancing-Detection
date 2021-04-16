#include "tek5030/stereo_pair.h"
#include "opencv2/imgproc.hpp"

namespace tek5030
{
namespace font
{
constexpr auto face = cv::FONT_HERSHEY_PLAIN;
constexpr auto scale = 1.0;
}
cv::Mat StereoPair::hconcat() const
{
  cv::Mat result;
  cv::hconcat(left, right, result);

  cv::putText(result, "LEFT", {10, 20}, font::face, font::scale, cv::Scalar::all(0));
  cv::putText(result, "RIGHT", {result.cols /2 + 10, 20}, font::face, font::scale, cv::Scalar::all(0));

  return result;
}
}
