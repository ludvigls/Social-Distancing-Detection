#include "sparse_stereo_matcher.h"

SparseStereoMatcher::SparseStereoMatcher(cv::Ptr<cv::Feature2D> detector,
                                         cv::Ptr<cv::Feature2D> desc_extractor)
    : detector_{std::move(detector)}
    , desc_extractor_{std::move(desc_extractor)}
    , matcher_{cv::makePtr<cv::BFMatcher>(desc_extractor_->defaultNorm())}
{}

void SparseStereoMatcher::match(const tek5030::StereoPair& stereo_img)
{
  const int n_grid_cols = stereo_img.left.cols / 16;
  const int n_grid_rows = stereo_img.left.rows / 16;
  const int patch_width = 32;

  // Detect and describe features in the left image.
  keypoints_left_.clear();
  keypoints_left_ = detectInGrid(stereo_img.left, detector_, {n_grid_cols, n_grid_rows}, 1, patch_width);
  cv::Mat query_descriptors;
  desc_extractor_->compute(stereo_img.left, keypoints_left_, query_descriptors);

  // Detect and describe features in the right image.
  keypoints_right_.clear();
  keypoints_right_ = detectInGrid(stereo_img.right, detector_, {n_grid_cols, n_grid_rows}, 1, patch_width);
  cv::Mat train_descriptors;
  desc_extractor_->compute(stereo_img.right, keypoints_right_, train_descriptors);

  // Match features
  std::vector<cv::DMatch> matches;
  matcher_->match(query_descriptors, train_descriptors, matches);

  extractGoodMatches(matches);
  computeDisparities();
}

void SparseStereoMatcher::extractGoodMatches(const std::vector<cv::DMatch>& matches)
{
  good_matches_.clear();

  for (const auto& match : matches)
  {
    const cv::Point2f& left_point = keypoints_left_[match.queryIdx].pt;
    const cv::Point2f& right_point = keypoints_right_[match.trainIdx].pt;
    // TODO (1/7): Exploit the stereo geometry!
    bool epipolar_is_ok =std::abs(left_point.y-right_point.y)<1.0;        // Do something more useful!
    bool disparity_is_positive = left_point.x-right_point.x>0; // Do something more useful!

    if (epipolar_is_ok && disparity_is_positive)
    {
      good_matches_.push_back(match);
    }
  }
}

void SparseStereoMatcher::computeDisparities()
{
  point_disparities_.clear();
  for (const auto& match : good_matches_)
  {
    const cv::Point2f& left_point = keypoints_left_[match.queryIdx].pt;
    const cv::Point2f& right_point = keypoints_right_[match.trainIdx].pt;

    // TODO (2/7): Compute disparity.
    const auto disparity = left_point.x-right_point.x; // Do something else!

    point_disparities_.emplace_back(left_point.x, left_point.y, disparity);
  }
}

std::vector<cv::KeyPoint> SparseStereoMatcher::detectInGrid(const cv::Mat& image, cv::Ptr<cv::Feature2D> detector, cv::Size grid_size, int max_in_cell, int patch_width)
{
  std::vector<cv::KeyPoint> all_keypoints;

  const int height = image.rows / grid_size.height;
  const int width = image.cols / grid_size.width;

  const int patch_rad = patch_width / 2;

  for (int x=0; x<grid_size.width; ++x)
  {
    for (int y=0; y<grid_size.height; ++y)
    {
      cv::Range row_range(std::max(y*height - patch_rad, 0), std::min((y+1)*height + patch_rad, image.rows));
      cv::Range col_range(std::max(x*width - patch_rad, 0), std::min((x+1)*width + patch_rad, image.cols));

      cv::Mat sub_img = image(row_range, col_range);

      std::vector<cv::KeyPoint> grid_keypoints;
      detector->detect(sub_img, grid_keypoints);

      cv::KeyPointsFilter::retainBest(grid_keypoints, max_in_cell);

      for (auto& keypoint : grid_keypoints)
      {
        keypoint.pt.x += col_range.start;
        keypoint.pt.y += row_range.start;
      }

      all_keypoints.insert(all_keypoints.end(), grid_keypoints.begin(), grid_keypoints.end());
    }
  }

  return all_keypoints;
}
