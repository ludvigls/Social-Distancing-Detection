#include "viewer_3d.h"

Viewer3D::Viewer3D()
  : scale_{0.02}
  , viz_win_{"3D Visualization"}
{
  viz_win_.showWidget("Left-axes", cv::viz::WCoordinateSystem(scale_));
  viz_win_.setViewerPose({cv::Matx33d::eye(), cv::Vec3d{0, 0, -0.5}});
  spinOnce();
}

void Viewer3D::addCameraFrustum(
    const std::string& camera_id,
    const cv::Matx33d& K,
    const cv::Mat& img,
    const cv::Affine3d& pose
)
{
  viz_win_.showWidget(
      camera_id,
      cv::viz::WCameraPosition(
          K,
          img,
          scale_
          ),
          pose);
}

void Viewer3D::spinOnce()
{
  viz_win_.spinOnce();
}

void Viewer3D::spin()
{
  viz_win_.spin();
}

void Viewer3D::addPointCloud(
    const std::vector<cv::Vec3d>& world_points,
    const std::vector<uint8_t>& colors
)
{
  cv::viz::WCloud pointcloud(world_points,colors);
  pointcloud.setRenderingProperty(cv::viz::POINT_SIZE, 4);
  viz_win_.showWidget("points", pointcloud);
}
