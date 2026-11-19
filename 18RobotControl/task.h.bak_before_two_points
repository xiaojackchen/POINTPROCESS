#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "img_lines.h"//已经引入img_process.h(引入img_data.h, img_thinning.h), img_roi.h


using namespace cv;
using namespace std;

class ImgTask { 
public:
bool firstFrameProcess(ImgData& data);
bool FrameProcess(ImgData& data);
void roiProcess(ImgData& data);
void InterPointUpdate(ImgData& data);
cv::Point2d getIntersection(ImgData& data, cv::Mat input, Point2d default_point);
cv::Point2d getIntersectionKF(ImgData& data, cv::Mat input, Point2d default_point);
cv::Point2d HoughgetIntersection(ImgData& data, cv::Mat input, Point2d default_point);
	void Reset();  // 重置首帧和 ROI 状态，用于重新加载视频
private:
	bool findHoughIntersections(ImgData& data);  // 从 long_lines 中求两组直线的交点
	int m_count = 0;    // 首帧计数
	int m_countKF = 0;  // 带卡尔曼滤波分支的首帧计数
};