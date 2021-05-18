#include "lab_7.h"
#include "cv_stereo_matcher_wrap.h"
#include "sparse_stereo_matcher.h"
#include "stereo_calibration.h"
#include "viewer_3d.h"
#include "visualization.h"
#include "tek5030/kitti_camera.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/stereo.hpp"
#include <math.h>       /* tan */
#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <sstream>
#include <fstream>

using namespace tek5030;
float dist(const Person &person1,const Person &person2){
  return pow(pow(person1.x-person2.x,2)+pow(person1.y+person2.y,2)+pow(person1.z+person2.z,2),1/2.0);
}
void addColor(std::vector<Person> &persons){
  
      for(auto it = std::begin(persons); it != std::end(persons); ++it) {
        float min_dist = 100000;
        for(auto it_inner = it; it_inner != std::end(persons); ++it_inner){
          if (abs(it->u-it_inner->u)>1 && abs(it->v-it_inner->v)>1){
            float dist_i=dist(*it,*it_inner);
            if (dist_i<min_dist){
              min_dist=dist_i;
            }
          }
        }
        if  (min_dist>2000.0){
          it->color=cv::viz::Color::green();
        } 
        else if (min_dist>1000.0){
          it->color=cv::viz::Color::yellow();
        }
        else{
          it->color=cv::viz::Color::red();
        }
        it->dist_nearest_person=min_dist/1000;
      }
      
}
void findXY(Person &person){
  // finds the translation between the person and the camera in 3D
  // input: a person object which has pixel coordinates in 2D
  // output: modifies the inputted person object to also contain translation between cam and person in 3D
  const float fov_hor=1.57079;
  const float fov_vert=0.61086;
  int width=1392;
  int height=512;
  int u_tilde=person.u-height/2;
  int v_tilde=person.v-width/2;

  float delta_deg_v=fov_hor/width;
  float delta_deg_u=fov_vert/height;

  float deg_v=delta_deg_v*v_tilde;
  float deg_u=delta_deg_u*u_tilde;

  person.x=person.z*sin(deg_u);
  person.y=person.z*sin(deg_v);
}

void lab7()
{
  
  std::ifstream ifs("points.json");
	Json::Reader reader;
	Json::Value obj;
	reader.parse(ifs, obj);

  // TODO 0: Fill in correct paths to the kitti dataset.
  const std::string dataset_path{"2011_09_28_drive_0045_extract/2011_09_28/2011_09_28_drive_0045_extract"};
  const std::string calib_path{"2011_09_28_calib/2011_09_28"};
  const bool color = false;
  
  KittiCamera camera(dataset_path, calib_path, color);

  cv::Ptr<cv::Feature2D> detector = cv::FastFeatureDetector::create();
  cv::Ptr<cv::Feature2D> desc_extractor = cv::BRISK::create(30, 0);

  SparseStereoMatcher stereo_matcher{detector, desc_extractor};

  const StereoCalibration calibration(camera);

  const std::string matching_win{"Stereo matching"};
  const std::string depth_win{"Stereo depth"};
  const std::string dense_win{"Dense disparity"};
  // cv::namedWindow(matching_win, cv::WINDOW_NORMAL);
  cv::namedWindow(depth_win, cv::WINDOW_NORMAL);
  cv::namedWindow(dense_win, cv::WINDOW_NORMAL);
  
  /*
  std::map<std::string, std::vector<std::vector<int>>> points; 
  std::vector<std::vector<int>> mat {{212, 270}, {584, 270}, {497, 267}, {352, 261}, {142, 260}};
  points["image_00000005_0.png"]=mat;
  */
  //Viewer3D viewer_3d;

  for (int i = 0;i<10000;i++)
  {
  

    const StereoPair stereo_raw = camera.getStereoPair();

    // Rectify images.
    const StereoPair stereo_rectified = calibration.rectify(stereo_raw);

    // Add frustums to the 3D visualization
    //viewer_3d.addCameraFrustum("Camera-frustum-left",  calibration.K_left(),  stereo_rectified.left);
    //viewer_3d.addCameraFrustum("Camera-frustum-right", calibration.K_right(), stereo_rectified.right, calibration.pose());

    // Perform sparse matching.
    // TODO (1/7): Make SparseStereoMatcher::extractGoodMatches() better!
    // TODO (2/7): Compute disparity in SparseStereoMatcher::computeDisparities()
    stereo_matcher.match(stereo_rectified);

    // Visualize matched point correspondences
    //const cv::Mat match_image = visualizeMatches(stereo_rectified, stereo_matcher);
    //cv::imshow(matching_win, match_image);

    // Get points with disparities.

    if (!stereo_matcher.point_disparities().empty())
    {
      cv::Mat visualized_depths;
      cv::cvtColor(stereo_rectified.left, visualized_depths, cv::COLOR_GRAY2BGR);

      std::vector<uint8_t> point_colors;
      
      cv::Ptr<cv::stereo::StereoMatcher> ptr = cv::stereo::StereoBinarySGBM::create(0, 128, 5); //mindisp, numdisp (16), blocksize(3)
      CvStereoMatcherWrap dense_matcher(ptr);
      const auto dense_disparity = dense_matcher.compute(stereo_rectified);
      cv::Mat dense_depth = calibration.f() * (calibration.baseline() / dense_disparity); // dense_disparity;

      //Viz dense depth image
      constexpr float max_depth = 50.f;
      dense_depth.setTo(0, (dense_disparity < 0) | (dense_depth > max_depth));
      dense_depth /= max_depth;

      cv::cvtColor(dense_depth, dense_depth, cv::COLOR_GRAY2BGR);
      dense_depth.convertTo(dense_depth, CV_8U, 255);
      cv::applyColorMap(dense_depth, dense_depth, cv::COLORMAP_JET);

      cv::imshow(dense_win, dense_depth);

      cv::Size s = dense_depth.size();
      //std::cout<<s.height<<std::endl;
      //std::cout<<s.width<<std::endl;

      std::vector<Person> persons;
      std::string file_name;
      if (i<10){
        file_name="image_0000000"+std::to_string(i)+"_0.png";}
      else if (i<100){
        file_name="image_000000"+std::to_string(i)+"_0.png";}
      else {
        file_name="image_00000"+std::to_string(i)+"_0.png";
      }
      //std::cout<<file_name<<std::endl;
      for (auto& point : obj[file_name]){
        const int u=static_cast<int>(std::round(point[0].asInt()));
        const int v=static_cast<int>(std::round(point[1].asInt()));
        //const auto disparity = d_vec[2];
        const auto depth =  calibration.f() * (calibration.baseline() / dense_depth.at<float>(v,u)); //u and v had to switch places, dont ask me why
        Person person;
        person.u=u;
        person.v=v;
        person.z=depth;
        findXY(person);
        persons.push_back(person);
      }
      /*
      for (const auto& d_vec : stereo_matcher.point_disparities())
      {
        const int u=static_cast<int>(std::round(d_vec[0]));
        const int v=static_cast<int>(std::round(d_vec[1]));
        // const auto disparity = d_vec[2];
        const auto depth =  calibration.f() * (calibration.baseline() / dense_depth.at<float>(v,u)); //TODO, u and v had to switch places, dont ask me why
        Person person;
        person.u=u;
        person.v=v;
        person.z=depth;
        findXY(person);
        persons.push_back(person);
      }
      */
      /*
      Person person;
      person.u=200;
      person.v=200;
      person.z=4;
      Person person1;
      person1.u=400;
      person1.v=400;
      person1.z=30;
      std::vector<Person> persons;
      persons.push_back(person);
      persons.push_back(person1);
      */
      addColor(persons);
      
      for(auto it = std::begin(persons); it != std::end(persons); ++it){
        //std::cout<<it->color<<std::endl;
        const cv::Point pixel_pos{
           it->u,it->v
        };
        addDepthPoint(visualized_depths, pixel_pos,it->z,it->color,it->dist_nearest_person);        // TODO (5/7): Get point intensity from corresponding pixel in stereo_rectified.left.
        //point_colors.push_back(stereo_rectified.left.at<uint8_t>(it->u, it->v));
      }
      
      cv::imshow(depth_win, visualized_depths);

      // TODO (4/7): Use cv::perspectiveTransform() to compute the 3D point cloud.
      /*

      std::vector<cv::Vec3d> world_points(point_colors.size());
                  
      cv::perspectiveTransform(stereo_matcher.point_disparities(), world_points, calibration.Q());

      viewer_3d.addPointCloud(world_points, point_colors);
    }

   {  // Dense stereo matching using OpenCV
      // TODO (6/7): Create a cv::Ptr<cv::stereo::StereoMatcher>

      cv::Ptr<cv::stereo::StereoMatcher> ptr = cv::stereo::StereoBinarySGBM::create(0, 128, 5); //mindisp, numdisp (16), blocksize(3)

      CvStereoMatcherWrap dense_matcher(ptr);
      const auto dense_disparity = dense_matcher.compute(stereo_rectified);
      if (!dense_disparity.empty())
      {
        // TODO (7/7): Compute depth in meters. Hint: same as todo 3, but without a loop
        cv::Mat dense_depth = calibration.f() * (calibration.baseline() / dense_disparity); // dense_disparity;

        constexpr float max_depth = 50.f;
        dense_depth.setTo(0, (dense_disparity < 0) | (dense_depth > max_depth));
        dense_depth /= max_depth;

        cv::cvtColor(dense_depth, dense_depth, cv::COLOR_GRAY2BGR);
        dense_depth.convertTo(dense_depth, CV_8U, 255);
        cv::applyColorMap(dense_depth, dense_depth, cv::COLORMAP_JET);

        cv::imshow(dense_win, dense_depth);
      }
    */
    }
    
    //viewer_3d.spinOnce();

    if (cv::waitKey(1) >= 0)
    { break; }
  }
}


