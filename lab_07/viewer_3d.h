#pragma once

#include "opencv2/viz.hpp"

/// \brief Visualize 3D data using OpenCV viz module
class Viewer3D
{
public:
  Viewer3D();

  /// \brief Display a camera frustum in the viewer
  /// \param camera_id A unique name for the camera frame
  /// \param K The intrinsics of the camera
  /// \param img an image to display inside the frustum
  /// \param pose The pose of the camera frame
  void addCameraFrustum(
      const std::string& camera_id,
      const cv::Matx33d& K,
      const cv::Mat& img,
      const cv::Affine3d& pose = cv::Affine3d::Identity()
  );

  /// \brief Display a point cloud in the viewer
  /// \param world_points The point cloud
  /// \param colors The color for each point. Must be the same number of values as points in world_points.
  void addPointCloud(const std::vector<cv::Vec3d>& world_points, const std::vector<uint8_t>& colors);

  /// \brief Update the viewer once.
  void spinOnce();

  /// \brief Update the viewer continuously.
  void spin();

private:
  double scale_;
  cv::viz::Viz3d viz_win_;
};
