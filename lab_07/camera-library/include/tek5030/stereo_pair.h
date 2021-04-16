#pragma once

#include "opencv2/core.hpp"

namespace tek5030
{
struct StereoPair
{
  cv::Mat left;
  cv::Mat right;

  [[nodiscard]] cv::Mat hconcat() const;
};

struct StampedStereoPair
{
  StereoPair stereo_pair;
  double usec_timestamp;
};
}
