#pragma once

#include "opencv2/core.hpp"
#include <jsoncpp/json/json.h>

//#include <json/json.h>

/// Perform sparse and dense stereo matching on images captured from a StereoCamera.
void personPlacer();

class Person {
  public:
    int u;
    int v;
    double z;
    float x;
    float y;
    cv::Scalar color;
    float dist_nearest_person;
};
void findXY(Person &person);