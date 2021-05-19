#pragma once

#include "opencv2/core.hpp"
#include <jsoncpp/json/json.h>
#include "stereo_calibration.h"
//#include <json/json.h>

/// Perform sparse and dense stereo matching on images captured from a StereoCamera.
void lab7();

class Person {
  public:
    int u;
    int v;
    double z;
    float x;
    float y;
    double depth;//distance from camera
    cv::Scalar color;
    float dist_nearest_person;
};
void findXYZ(Person &person,const StereoCalibration calibration);