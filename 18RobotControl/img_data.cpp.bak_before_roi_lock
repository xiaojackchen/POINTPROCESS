#include "img_data.h"
#include <algorithm>
#include <cmath>

ImgData::ImgData() {};
ImgData::~ImgData() {};

void ImgData::clear(){
	// 固定 ROI：手工标定的一块区域，x 范围 [241, 949]，y 范围 [0, 720]
	// 超出画面的部分自动裁掉，保证后续 inputcolor(rect)、copyTo(rect) 都不越界
	if (!inputcolor.empty()) {
		static const cv::Rect kFixedRoi(241, 0, 949 - 241, 720);
		const cv::Rect rrFixed = kFixedRoi & cv::Rect(0, 0, inputcolor.cols, inputcolor.rows);
		if (rrFixed.width > 0 && rrFixed.height > 0) {
			// 固定 ROI 每帧只留一条，避免累积同一条框造成重复绘制、重复打印
			roi_region.clear();
			roi_region.push_back(rrFixed);
		}
	}
	/*roi_region.clear();*/

    inputcolor = cv::Mat();
    
    binary = cv::Mat();
    bilFilt_img = cv::Mat();
    input2 = cv::Mat();
    skeleton = cv::Mat();
    global_skeleton = cv::Mat();
    input_roicolor = cv::Mat();
    vertical_line = cv::Vec4i();
    incline_line = cv::Vec4i();
    lineEquation_vertical = LineEquation();
    lineEquation_incline = LineEquation();
    vertical_line2 = cv::Vec4i();
    incline_line2 = cv::Vec4i();
    lineEquation_vertical2 = LineEquation();
    lineEquation_incline2 = LineEquation();
    intersection2 = cv::Point2d();
    upper_line = cv::Vec4f();
    lower_line = cv::Vec4f();
    intersection = cv::Point2d();
    long_lines.clear();
    upper_mask = cv::Mat();
    lower_mask = cv::Mat();
    upper_roi = cv::Mat();
    lower_roi = cv::Mat();
    final_lines1.clear();
    final_lines2.clear();
}