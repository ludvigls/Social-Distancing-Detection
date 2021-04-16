#pragma once

#include "tek5030/stereo_pair.h"
#include "opencv2/features2d.hpp"

/// \brief Perform stereo processing using keypoints in images
class SparseStereoMatcher
{
public:
  /// \brief Detect and match keypoints in both images.
  /// This function will call extracGoodMatches and computeDisparities.
  /// \param stereo_img Input images.
  /// \see extractGoodMatches
  /// \see computeDisparities
  void match(const tek5030::StereoPair& stereo_img);

  /// \brief Create a new SparseStereoMatcher
  /// \param detector The feature detector to use (see lab 3 and 4)
  /// \param desc_extractor The feature descriptor to use (see lab 3 and 4)
  /// \param max_num_features The number of feature points to extract
  /// \param max_ratio max ratio for the ratio test (finding unique matches)
  SparseStereoMatcher(cv::Ptr<cv::Feature2D> detector,
                      cv::Ptr<cv::Feature2D> desc_extractor);

  /// \return keypoints detected in the left image
  const std::vector<cv::KeyPoint>& keypoints_left() const
  { return keypoints_left_; }

  /// \return keypoints detected in the right image
  const std::vector<cv::KeyPoint>& keypoints_right() const
  { return keypoints_right_; }

  /// \return matched keypoints (keypoint pairs in left and right images)
  const std::vector<cv::DMatch>& matches() const
  { return good_matches_; }

  /// \return computed disparity values for each keypoint pair in matches
  /// \see mathces
  const std::vector<cv::Vec3d>& point_disparities() const
  { return point_disparities_; }

private:
  /// \brief compute disparity values for each keypoint pair in good_matches_
  void computeDisparities();

  /// \brief Perform epipolar test to keep only the good keypoint correspondences.
  void extractGoodMatches(const std::vector<cv::DMatch>& matches);

  /// \brief Detect keypoints in a grid.
  std::vector<cv::KeyPoint> detectInGrid(
      const cv::Mat& image,
      cv::Ptr<cv::Feature2D> detector,
      cv::Size grid_size,
      int max_in_cell,
      int patch_width);

  cv::Ptr<cv::Feature2D> detector_;
  cv::Ptr<cv::Feature2D> desc_extractor_;

  cv::Ptr<cv::DescriptorMatcher> matcher_;
  std::vector<cv::KeyPoint> keypoints_left_;
  std::vector<cv::KeyPoint> keypoints_right_;
  std::vector<cv::DMatch> good_matches_;

  std::vector<cv::Vec3d> point_disparities_;
};
