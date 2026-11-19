#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class img_thinning{
public:
    void zhangSuenThinning(cv::Mat& src, cv::Mat& dst);

public:
    img_thinning();
    ~img_thinning();
};
