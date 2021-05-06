#include "tek5030/kitti_camera.h"

#include <fstream>
#include <iostream>
#include <regex>

namespace tek5030
{
namespace kitti
{
static constexpr const char* camera_folder = "image_";
static constexpr const char* data_folder = "data";
static constexpr const char* fname_calib_cam_to_cam = "calib_cam_to_cam.txt";
static constexpr const char* fname_timestamps = "timestamps.txt";
static constexpr const char* gray_left_cam = "00";
static constexpr const char* gray_right_cam = "01";
static constexpr const char* color_left_cam = "02";
static constexpr const char* color_right_cam = "03";

std::map<std::string, KittiCamera::Calibration> loadCamToCamCalibration(const std::string& fname);

std::vector<double> parseVectorOfDoubles(const std::string& input);
}

KittiCamera::KittiCamera(
  const std::string& dataset_path,
  const std::string& calib_path,
  const bool color
)
  : color_{color}
  , left_cap_{dataset_path + '/' + kitti::camera_folder
              + (color ? kitti::color_left_cam : kitti::gray_left_cam)
              + '/' + kitti::data_folder + '/' + "image_0%08d_0.png"}
  , right_cap_{dataset_path + '/' + kitti::camera_folder
               + (color ? kitti::color_right_cam : kitti::gray_right_cam)
               + '/' + kitti::data_folder + '/' + "image_0%08d_1.png"}
  , calibration_map_{kitti::loadCamToCamCalibration(calib_path + '/' + kitti::fname_calib_cam_to_cam)}
{
  std::cout << "path: " << std::string{dataset_path + '/' + kitti::camera_folder
                            + (color ? kitti::color_left_cam : kitti::gray_left_cam)
                            + '/' + kitti::data_folder + '/' + "0%09d.png"} << std::endl;
  if (!left_cap_.isOpened())
  { throw std::invalid_argument("Could not open left camera directory"); }

  if (!right_cap_.isOpened())
  { throw std::invalid_argument("Could not open right camera directory"); }
}

StereoPair KittiCamera::getStereoPair() const
{
  // Use VideoCapture::grab() to make sure that the images are captured as near in time as possible.
  left_cap_.grab();
  right_cap_.grab();

  StereoPair stereo_pair{};
  // The captured images are retrieved with VideoCapture::retrieve().
  const bool left_ok = left_cap_.retrieve(stereo_pair.left);
  const bool right_ok = right_cap_.retrieve(stereo_pair.right);

  if (!left_ok)
  { throw std::runtime_error{"End of left camera stream"}; }

  if (!right_ok)
  { throw std::runtime_error{"End of right camera stream"}; }

  // Seems to be a change in OpenCV 4.4.0
  if (!color_ && stereo_pair.left.channels() == 3)
  {
    cv::Mat bgr[3];
    cv::split(stereo_pair.left, bgr);
    stereo_pair.left = bgr[0];
  }

  if (!color_ && stereo_pair.right.channels() == 3)
  {
    cv::Mat bgr[3];
    cv::split(stereo_pair.right, bgr);
    stereo_pair.right = bgr[0];
  }
  ++frame_count_;

  return stereo_pair;
}

size_t KittiCamera::getFrameCount() const
{
  return frame_count_;
}

KittiCamera::Calibration KittiCamera::getCalibration(KittiCamera::Cam cam) const
{
  switch (cam)
  {
    case Cam::GrayLeft:
      return calibration_map_.at(kitti::gray_left_cam);
    case Cam::GrayRight:
      return calibration_map_.at(kitti::gray_right_cam);
    case Cam::ColorLeft:
      return calibration_map_.at(kitti::color_left_cam);
    case Cam::ColorRight:
      return calibration_map_.at(kitti::color_right_cam);
  }
  throw std::invalid_argument("unexpected error");
}

bool KittiCamera::color() const
{
  return color_;
}

/// Sensor Calibration
/// ==================
///
/// The sensor calibration zip archive contains files, storing matrices in
/// row-aligned order, meaning that the first values correspond to the first
/// row:
///
/// calib_cam_to_cam.txt: Camera-to-camera calibration
/// --------------------------------------------------
///
///   - S_xx: 1x2 size of image xx before rectification
///   - K_xx: 3x3 calibration matrix of camera xx before rectification
///   - D_xx: 1x5 distortion vector of camera xx before rectification
///   - R_xx: 3x3 rotation matrix of camera xx (extrinsic)
///   - T_xx: 3x1 translation vector of camera xx (extrinsic)
///   - S_rect_xx: 1x2 size of image xx after rectification
///   - R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
///   - P_rect_xx: 3x4 projection matrix after rectification
///
/// Note: When using this dataset you will most likely need to access only
/// P_rect_xx, as this matrix is valid for the rectified image sequences.
std::map<std::string, KittiCamera::Calibration> kitti::loadCamToCamCalibration(const std::string& fname)
{
  std::ifstream import_file(fname, std::ios::in);

  if (!import_file)
  { throw std::invalid_argument("Could not open '" + fname + "'"); }

  std::map<std::string, KittiCamera::Calibration> calibrations;

  std::string line;
  while (std::getline(import_file, line))
  {
    std::stringstream line_stream(line);

    // Check what the header is. Each line consists of two parts:
    // a header followed by a ':' followed by space-separated data.
    std::string header, data;
    std::getline(line_stream, header, ':');
    std::getline(line_stream, data, ':');

    const std::regex header_regex(R"((\w)_(rect_)?(\d\d))");
    std::smatch matches;

    if (std::regex_match(header, matches, header_regex))
    {
      const auto cam_idx = matches[3].str();
      const auto field = matches[1].str();
      const auto rectified = matches[2].matched;
      const auto parsed_doubles = parseVectorOfDoubles(data);

      if (matches[1].compare("S") == 0)
      {
        if (parsed_doubles.size() != 2)
        { throw std::runtime_error("parsed_doubles has incorrect length"); }

        const cv::Size2d size(parsed_doubles[0], parsed_doubles[1]);

        if (rectified)
        { calibrations[cam_idx].rectified.size = size; }
        else
        { calibrations[cam_idx].size = size; }
      } else if (matches[1].compare("K") == 0)
      {
        cv::Mat K(3, 3, CV_64F);

        if (parsed_doubles.size() != K.total())
        { throw std::runtime_error("parsed_doubles has incorrect length"); }

        std::memcpy(K.data, parsed_doubles.data(), K.total() * K.elemSize());
        calibrations[cam_idx].K = K;
      } else if (matches[1].compare("D") == 0)
      {
        cv::Mat d(1, 5, CV_64F);

        if (parsed_doubles.size() != d.total())
        { throw std::runtime_error("parsed_doubles has incorrect length"); }

        std::memcpy(d.data, parsed_doubles.data(), d.total() * d.elemSize());
        calibrations[cam_idx].d = d;
      } else if (matches[1].compare("R") == 0)
      {
        cv::Mat R(3, 3, CV_64F);

        if (parsed_doubles.size() != R.total())
        { throw std::runtime_error("parsed_doubles has incorrect length"); }

        std::memcpy(R.data, parsed_doubles.data(), R.total() * R.elemSize());

        if (rectified)
        { calibrations[cam_idx].rectified.R = R; }
        else
        { calibrations[cam_idx].rotation = R; }
      } else if (matches[1].compare("T") == 0)
      {
        cv::Mat t(1, 3, CV_64F);

        if (parsed_doubles.size() != t.total())
        { throw std::runtime_error("parsed_doubles has incorrect length"); }

        std::memcpy(t.data, parsed_doubles.data(), t.total() * t.elemSize());
        calibrations[cam_idx].translation = t;
      } else if (matches[1].compare("P") == 0)
      {
        if (!rectified)
        { throw std::runtime_error("P should be _rect_"); }

        cv::Mat P(3, 4, CV_64F);

        if (parsed_doubles.size() != P.total())
        { throw std::runtime_error("parsed_doubles has incorrect length"); }

        std::memcpy(P.data, parsed_doubles.data(), P.total() * P.elemSize());
        calibrations[cam_idx].rectified.P = P;
      }
    }
  }

  return calibrations;
}

std::vector<double> kitti::parseVectorOfDoubles(const std::string& input)
{
  std::stringstream line_stream(input);
  if (line_stream.eof())
  { throw std::invalid_argument("could not read input"); }

  std::vector<double> parsed_data;
  while (!line_stream.eof())
  {
    std::string element;
    std::getline(line_stream, element, ' ');
    if (element.empty())
    { continue; }

    parsed_data.emplace_back(std::stod(element));
  }
  return parsed_data;
}
}
