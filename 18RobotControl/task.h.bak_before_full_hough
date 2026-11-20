#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "img_lines.h"//已经引入img_process.h(引入img_data.h, img_thinning.h), img_roi.h


using namespace cv;
using namespace std;

class ImgTask { 
public:
	// 每帧返回两个交点：pts[0] = 第一交点(75 度直线 ∩ 远侧近垂直线)，pts[1] = 第二交点(-75 度直线 ∩ 近侧近垂直线)
	// 某个点无效时该元素为 default_point，可用 p.x > 0 && p.y > 0 判断是否有效
	std::vector<cv::Point2d> getIntersection(ImgData& data, cv::Mat input, Point2d default_point);
	std::vector<cv::Point2d> getIntersectionKF(ImgData& data, cv::Mat input, Point2d default_point);
	std::vector<cv::Point2d> HoughgetIntersection(ImgData& data, cv::Mat input, Point2d default_point);

	bool firstFrameProcess(ImgData& data);        // 首帧：整帧预处理 → 检测直线 → 求两个交点
	bool FrameProcess(ImgData& data);             // 用 global_skeleton 再做一次 Hough 检测（异常时兜底）
	void roiProcess(ImgData& data);               // ROI 局部预处理 + 两个交点各自的 RANSAC 精修
	void InterPointUpdate(ImgData& data);         // 两个点各自校验：越界/跳变 → FrameProcess 兜底 → 回退上一帧
	bool findHoughIntersections(ImgData& data);   // 从 long_lines 中求两组直线的交点
	void Reset();  // 重置首帧和 ROI 状态，用于重新加载视频

private:
	std::vector<cv::Point2d> collectPoints(const ImgData& data, const cv::Point2d& default_point) const;
	static bool isPointValid(const cv::Point2d& p, const cv::Mat& img);

	int m_count = 0;    // 首帧计数
	int m_countKF = 0;  // 带卡尔曼滤波分支的首帧计数
};
