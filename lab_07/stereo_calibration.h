#pragma once

#include "tek5030/stereo_pair.h"
#include "tek5030/kitti_camera.h"
#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"

class StereoCalibration
{
public:
  /// \brief Load stereo calibration parameters from files.
  /// \param intrinsic_filename *.yml file containing intrinsics.
  /// \param extrinsic_filename *.yml file containing extrinsics.
  /// \param img_size The image size for which the calibration is valid.
  StereoCalibration(const std::string& intrinsic_filename,
                    const std::string& extrinsic_filename,
                    const cv::Size& img_size);

  /// \brief Load stereo calibration parameters from a RealSense camera.
  /// \param stereo_camera The camera.
  explicit StereoCalibration(const tek5030::KittiCamera& stereo_camera);

  /// \brief Rectify a stereo pair with the loaded calibration parameters.
  /// \param raw_stereo_pair The input images
  /// \return rectified images, ready for stereo processing
  tek5030::StereoPair rectify(const tek5030::StereoPair& raw_stereo_pair) const;

  /// \return the distance between the origins of the cameras
  double baseline() const
  { return 1.0 / Q_.at<double>(3, 2); }

  /// \return the f-coefficient of the Q-matrix
  double f() const
  { return Q_.at<double>(2, 3); }

  /// \return The image size for which the calibration is valid.
  cv::Size img_size() const
  { return img_size_; }

  /// \return the computed Q-matrix from the calibration parameters.
  const cv::Mat& Q() const
  { return Q_; }

  /// \return the intrinsic parameters of the left camera
  const cv::Mat& K_left() const
  { return K_left_; }

  /// \return the distortion parameters of the left camera
  const cv::Mat& distortion_left() const
  { return distortion_left_; }

  /// \return the intrinsic parameters of the right camera
  const cv::Mat& K_right() const
  { return K_right_; }

  /// \return the distortion parameters of the right camera
  const cv::Mat& distortion_right() const
  { return distortion_right_; }

  /// \return the rotation and translation of the left camera relative to the right camera.
  cv::Affine3d pose() const
  {
    cv::Mat t = -R_.t() * t_;
    return cv::Affine3d(R_.t(), t);
  }

private:
  void computeRectificationMapping();

  cv::Size img_size_;
  cv::Mat Q_;

  // Intrinsics.
  cv::Mat K_left_;
  cv::Mat distortion_left_;
  cv::Mat K_right_;
  cv::Mat distortion_right_;

  // Extrinsics.
  cv::Mat R_;
  cv::Mat t_;

  // Rectification mappings.
  cv::Mat map_left_x_;
  cv::Mat map_left_y_;
  cv::Mat map_right_x_;
  cv::Mat map_right_y_;
};
