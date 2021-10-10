#include "person_placer.h"
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
float euclidianDist(const Person &person1,const Person &person2){
  return pow(pow(person1.x-person2.x,2)+pow(person1.y+person2.y,2)+pow(person1.z+person2.z,2),1/2.0);
}

void addColor(std::vector<Person> &persons){
      // finds out if the closest person is either more than 2m away, between 1 and 2 m or less than 1 m
      // input: person objects including their 3D positions
      // output: 
      for(auto it = std::begin(persons); it != std::end(persons); ++it) {
        float min_dist = 10000000;
        for(auto it_inner = it; it_inner != std::end(persons); ++it_inner){
          if (abs(it->u-it_inner->u)>1 && abs(it->v-it_inner->v)>1){
            float dist_i=euclidianDist(*it,*it_inner);
            if (dist_i<min_dist){
              min_dist=dist_i;
            }
          }
        }
        if  (min_dist>2.0){
          it->color=cv::viz::Color::green();
        } 
        else if (min_dist>1.0){
          it->color=cv::viz::Color::yellow();
        }
        else{
          it->color=cv::viz::Color::red();
        }
        it->dist_nearest_person=min_dist/1.0; // dividing by a float casts it to float
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

void personPlacer()
{
  //loading the precomputed person points
  std::ifstream ifs("points1.json");
	Json::Reader reader;
	Json::Value obj;
	reader.parse(ifs, obj);

  // paths to the dataset
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
  const std::string dense_win{"Dense depth"};

  cv::namedWindow(depth_win, cv::WINDOW_NORMAL);
  cv::namedWindow(dense_win, cv::WINDOW_NORMAL);
  

  for (int i = 0;i<10000;i++) //iterate over image pair sequence
  {
    const StereoPair stereo_raw = camera.getStereoPair();

    // Rectify images.
    const StereoPair stereo_rectified = calibration.rectify(stereo_raw);

    stereo_matcher.match(stereo_rectified); 



    if (!stereo_matcher.point_disparities().empty())
    {
      cv::Mat visualized_depths;
      cv::cvtColor(stereo_rectified.left, visualized_depths, cv::COLOR_GRAY2BGR);

      std::vector<uint8_t> point_colors;
      
      cv::Ptr<cv::stereo::StereoMatcher> ptr = cv::stereo::StereoBinarySGBM::create(0, 128, 5); //mindisp, numdisp (16), blocksize(3)
      CvStereoMatcherWrap dense_matcher(ptr);
      const auto dense_disparity = dense_matcher.compute(stereo_rectified);
      cv::Mat dense_depth = calibration.f() * (calibration.baseline() / dense_disparity); 

      std::vector<Person> persons;
      std::string file_name;
      if (i<10){
        file_name="000000000"+std::to_string(i)+".png";}
      else if (i<100){
        file_name="00000000"+std::to_string(i)+".png";}
      else {
        file_name="0000000"+std::to_string(i)+".png";
      }
      for (auto& point : obj[file_name]){
        const int u=static_cast<int>(std::round(point[0].asInt()));
        const int v=static_cast<int>(std::round(point[1].asInt()));
        Person person;
        person.u=u;
        person.v=v;
        person.z=dense_depth.at<float>(v,u);
        std::cout<<"f: "<<calibration.f()<<" baseline"<<calibration.baseline()<<std::endl;
        std::cout<<person.z<<std::endl;
        findXY(person);
        persons.push_back(person);
      }
      
      addColor(persons);
      
      for(auto it = std::begin(persons); it != std::end(persons); ++it){
        //std::cout<<it->color<<std::endl;
        const cv::Point pixel_pos{
           it->u,it->v
        };
        addDepthPoint(visualized_depths, pixel_pos,it->z,it->color,it->dist_nearest_person);     
      }
      
      cv::imshow(depth_win, visualized_depths);

   
    }
    
    if (cv::waitKey(1) >= 0)
    { break; }
  }
}


