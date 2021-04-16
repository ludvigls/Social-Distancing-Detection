#include "stereo_calibration.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

StereoCalibration::StereoCalibration(const std::string& intrinsic_filename,
                                     const std::string& extrinsic_filename,
                                     const cv::Size& img_size)
    : img_size_{img_size}
{
  cv::FileStorage fs(intrinsic_filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::stringstream message;
    message << "Failed to open file " << intrinsic_filename;
    throw std::runtime_error{message.str()};
  }

  fs["M1"] >> K_left_;
  fs["D1"] >> distortion_left_;
  fs["M2"] >> K_right_;
  fs["D2"] >> distortion_right_;

  fs.open(extrinsic_filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::stringstream message;
    message << "Failed to open file " << extrinsic_filename;
    throw std::runtime_error{message.str()};
  }

  fs["R"] >> R_;
  fs["T"] >> t_;

  computeRectificationMapping();
}

StereoCalibration::StereoCalibration(const tek5030::KittiCamera& stereo_camera)
{
  const auto left = stereo_camera.getCalibration(stereo_camera.color() ? tek5030::KittiCamera::Cam::ColorLeft: tek5030::KittiCamera::Cam::GrayLeft);
  const auto right = stereo_camera.getCalibration(stereo_camera.color() ? tek5030::KittiCamera::Cam::ColorRight: tek5030::KittiCamera::Cam::GrayRight);

  K_left_ = left.K.clone();
  K_right_ = right.K.clone();;
  distortion_left_ = left.d.clone();;
  distortion_right_ = right.d.clone();;
  R_ = right.rotation.clone();;
  t_ = right.translation.t();;

  img_size_ = left.size;

  computeRectificationMapping();
}

void StereoCalibration::computeRectificationMapping()
{
  cv::Mat R_left;
  cv::Mat R_right;
  cv::Mat P_left;
  cv::Mat P_right;

  cv::stereoRectify(K_left_, distortion_left_,
                    K_right_, distortion_right_,
                    img_size_, R_, t_,
                    R_left, R_right, P_left, P_right,
                    Q_, cv::CALIB_ZERO_DISPARITY, -1, img_size_);

  cv::initUndistortRectifyMap(K_left_, distortion_left_, R_left, P_left,
                              img_size_, CV_16SC2, map_left_x_, map_left_y_);
  cv::initUndistortRectifyMap(K_right_, distortion_right_, R_right, P_right,
                              img_size_, CV_16SC2, map_right_x_, map_right_y_);
}

tek5030::StereoPair StereoCalibration::rectify(const tek5030::StereoPair& raw_stereo_pair) const
{
  tek5030::StereoPair rectified;

  if (raw_stereo_pair.left.type() == CV_16UC1)
  {
    raw_stereo_pair.left.convertTo(rectified.left, CV_8UC1, 255.0/65535.0);
    raw_stereo_pair.right.convertTo(rectified.right, CV_8UC1, 255.0/65535.0);
  }
  else
  {
    rectified = raw_stereo_pair;
  }

  remap(rectified.left, rectified.left, map_left_x_, map_left_y_, cv::INTER_LINEAR);
  remap(rectified.right, rectified.right, map_right_x_, map_right_y_, cv::INTER_LINEAR);
  return rectified;
}
