#pragma once

#include "tek5030/stereo_pair.h"
#include "opencv2/core.hpp"

class SparseStereoMatcher;

/// \brief This function will create an image that shows corresponding keypoints in two images.
/// \param stereo_pair The two images
/// \param stereo_matcher The matcher that has extracted the keypoints
/// \return an image with visualization of keypoint matches
cv::Mat visualizeMatches(const tek5030::StereoPair& stereo_pair, const SparseStereoMatcher& stereo_matcher);

/// \brief In an image, draw a cross at a pixel coordinate with a depth value printed next to it
/// \param visualized_depth The image to draw on
/// \param pos The pixel position of the depth measurement
/// \param depth The depth value
void addDepthPoint(cv::Mat& visualized_depth, const cv::Point& pos, double depth);
