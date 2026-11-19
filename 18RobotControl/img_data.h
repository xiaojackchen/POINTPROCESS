#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;
//#include "img_lines.h"//不能引用，会发生循环引用！！！

struct LineEquation {
    double a;
    double b;
    bool is_vertical = false;
    double x0;
};

class ImgData { 
public:
    ImgData();
    ~ImgData();
public:
    // img_process.h的成员变量
    cv::Mat inputcolor, gray, binary, equalized, bilFilt_img, input2, skeleton, global_skeleton, input_roicolor;
    vector<Rect> roi_region;
    // img_lines.h的成员变量
    std::vector<cv::Point2d> intersectionPoints, intersectionPoints1, intersectionPoints2, current_interPoint, previous_interPoint;
    Vec4i vertical_line, incline_line;
    LineEquation lineEquation_vertical, lineEquation_incline;
    // 第二组直线：86 度和 -75 度，用于计算第二个交点
    Vec4i vertical_line2, incline_line2;
    LineEquation lineEquation_vertical2, lineEquation_incline2;
    cv::Point2d intersection2;
    Vec4f upper_line, lower_line;
    cv::Point2d intersection;
    vector<Vec4i> long_lines;
    // img_roi.h的成员变量
    cv::Mat upper_mask, lower_mask;
    cv::Mat upper_roi, lower_roi;
    double angle1, angle2;

    // 清除data的部分数据
    void clear();
};
