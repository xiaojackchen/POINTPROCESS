#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class img_roi {
public:
img_roi() = default;
~img_roi() = default;

public:
bool createRoiMasks(cv::Mat& inputImg, cv::Mat& gobal_skeleton,
                      int upper_top, int upper_bottom,
                      int lower_top, int lower_bottom,
                      int x_upper_start, int x_upper_end,
                      int x_lower_start, int x_lower_end,
                      cv::Mat& upper_mask, cv::Mat& lower_mask,
                      cv::Mat& upper_roi, cv::Mat& lower_roi);
};
