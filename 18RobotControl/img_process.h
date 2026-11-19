#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "img_data.h"
#include "img_debug.h"
#include "img_thinning.h"

using namespace cv;
using namespace std;

class ImgProcess
{
public:
    ImgProcess() = default;
    ~ImgProcess() = default;
    
	// 双边滤波
	void Img_bilateralFilter(cv::Mat src, cv::Mat& dst,int count);

	// 自适应阈值处理
	void Img_adaptiveThreshold(cv::Mat& src, cv::Mat& dst);

    // 面积阈值处理
    void Img_areaThreshold(cv::Mat& src, cv::Mat& src_both, int area_thresh);

    // 开运算
    void Img_open(cv::Mat& src, cv::Mat& src_both, int count, int size);

    // 闭运算
    void Img_close(cv::Mat& src, cv::Mat& src_both, int count, int size);

	void Img_ellipse(cv::Mat& src, cv::Mat& src_both, int count, int size);
	void Img_dilate(cv::Mat& src, cv::Mat& src_both, int count, int size);

    // 预处理
    void preprocessImage(ImgData& data);
    void roipreprocessImage(ImgData& data);

    //将小尺寸图像复制到大尺寸图像中
    void copySmallImg(cv::Mat& src, cv::Mat& dst, int x, int y);
    void copySmallImg(cv::Mat& src, cv::Mat& dst, Size size, cv::Rect roi);
};
