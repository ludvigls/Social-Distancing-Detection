#pragma once

#include "opencv2/core.hpp"
#include <jsoncpp/json/json.h>

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
    cv::Scalar color;
};
void findXY(Person &person);