#ifndef COMMON_INCLUDE_H_
#define COMMON_INCLUDE_H_

#include <Eigen/Core>
#include <cmath>
#include <chrono>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
typedef Sophus::SE2d SE2;
typedef Sophus::SO2d SO2;

using namespace cv;
using namespace std;

#endif
