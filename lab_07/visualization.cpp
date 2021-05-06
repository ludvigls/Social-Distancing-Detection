#include "visualization.h"
#include "opencv2/imgproc.hpp"
#include "sparse_stereo_matcher.h"

#include <iomanip>

// Define a few font parameters for convenience.
namespace font
{
constexpr auto face = cv::FONT_HERSHEY_PLAIN;
constexpr auto scale = 1.0;
}

// Define a few BGR-colors for convenience.
namespace color
{
const cv::Scalar green(0, 255, 0);
const cv::Scalar red(0, 0, 255);
}

cv::Mat visualizeMatches(const tek5030::StereoPair& stereo_pair, const SparseStereoMatcher& stereo_matcher)
{
  cv::putText(stereo_pair.left, "LEFT", {10, 20}, font::face, font::scale, color::green);
  cv::putText(stereo_pair.right, "RIGHT", {10, 20}, font::face, font::scale, color::green);
  cv::Mat visualized_matches;
  cv::drawMatches(
      stereo_pair.left,
      stereo_matcher.keypoints_left(),
      stereo_pair.right,
      stereo_matcher.keypoints_right(),
      stereo_matcher.matches(),
      visualized_matches,
      color::green,
      color::red
  );
  return visualized_matches;
}

void addDepthPoint(cv::Mat& visualized_depth, const cv::Point& pos, const double depth)
{
  constexpr int marker_size = 5;
  cv::drawMarker(visualized_depth, pos, color::green, cv::MARKER_CROSS, marker_size);
  std::stringstream depth_text;
  depth_text << std::fixed << std::setprecision(2) << depth;
  cv::putText(visualized_depth, depth_text.str(), pos, font::face, font::scale, color::green);
}