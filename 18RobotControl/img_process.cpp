#include "img_process.h"

// 图像处理测试开关的定义
bool g_showImageDebug = false;
using namespace cv;
using namespace std;

/**
 * @brief 
 * src_both表示和src为同一个参数，既是输入也是输出
 */
void ImgProcess::Img_bilateralFilter(cv::Mat src,  cv::Mat& dst, int count) {
    for (int i = 0; i < count; i++) {
        cv::bilateralFilter(src, dst,
            /* d= */ 5,            // 邻域直径（建议5-15）
            /* sigmaColor= */ 30,   // 颜色空间滤波sigma（建议30-100）100
            /* sigmaSpace= */ 30);  // 坐标空间滤波sigma（建议30-100）5
        src = dst.clone();
    }
   
}

void ImgProcess::Img_adaptiveThreshold(cv::Mat& src, cv::Mat& dst){
    int adaptive_size = min(51, max(9, (min(src.cols, src.rows) / 50) | 0x01));// 确保blockSize在3-51之间，且为奇数
    //cv::adaptiveThreshold(src, dst, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, adaptive_size, -1);//(9,-5)还行
	cv::adaptiveThreshold(src, dst, 255, ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, adaptive_size, 0);//(9,-5)还行
   
}

void ImgProcess::Img_areaThreshold(cv::Mat& src, cv::Mat& src_both, int area_thresh){
    cv::Mat labels, stats, centroids;
    int numObjects = cv::connectedComponentsWithStats(src, labels, stats, centroids, 8, CV_32S);
    // 创建结果图像（默认全黑背景）
    cv::Mat result = cv::Mat::zeros(src.size(), CV_8UC1);
    // 遍历所有连通域（跳过背景0）
    for (int i = 1; i < numObjects; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        // 只保留大于阈值的区域
        if (area >= area_thresh) {
            // 创建当前区域的掩码
            cv::Mat mask = (labels == i);
            // 将保留的区域设置为白色
            result.setTo(255, mask);
        }
    }
    src_both = result.clone();
   // if (g_showImageDebug) cv::imshow("gray adaptiveThreshold TULV aera_thresh", src_both);
}

void ImgProcess::Img_open(cv::Mat& src, cv::Mat& src_both, int count, int size){
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
    for(int i = 0; i < count; i++){
        cv::morphologyEx(src, src_both, cv::MORPH_OPEN, element);
    }
    //if (g_showImageDebug) cv::imshow("preprocess: open (3x3)", src_both);
}

void ImgProcess::Img_close(cv::Mat& src, cv::Mat& src_both, int count, int size){
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
    for(int i = 0; i < count; i++){
        cv::morphologyEx(src, src_both, cv::MORPH_CLOSE, element);
    }
    if (g_showImageDebug) cv::imshow("preprocess: close (5x5 x10)", src_both);
}

void ImgProcess::Img_dilate(cv::Mat& src, cv::Mat& src_both, int count, int size) {
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
	for (int i = 0; i < count; i++) {
		cv::morphologyEx(src, src_both, cv::MORPH_DILATE, element);
	}
	//cv::imshow("gray adaptiveThreshold TULV aera_thresh close", src_both);
}

void ImgProcess::Img_ellipse(cv::Mat& src, cv::Mat& src_both, int count, int size) {
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
	for (int i = 0; i < count; i++) {
		cv::morphologyEx(src, src_both, cv::MORPH_ELLIPSE, element);
	}
	//cv::imshow("gray adaptiveThreshold TULV aera_thresh close", src_both);
}

void ImgProcess::preprocessImage(ImgData& data) {
	//if (g_showImageDebug) cv::imshow("original", data.inputcolor);
    // 1.双边滤波
    Img_bilateralFilter(data.inputcolor, data.bilFilt_img, 3);
    // 2.灰度化
    cv::cvtColor(data.bilFilt_img, data.gray, cv::COLOR_BGR2GRAY);
   // if (g_showImageDebug) cv::imshow("gray", data.gray);
    // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(40.0, Size(8, 8));
	// clahe->apply(data.gray, data.equalized);
	// cv::imshow("equalized", data.equalized);
    // 3.均值滤波
    cv::blur(data.gray, data.gray, cv::Size(5, 5));
   //if (g_showImageDebug) cv::imshow("gray Median Blurred", data.gray);
    // 4.自适应阈值
   double t = cv::threshold( data.gray,data.binary,0,255,THRESH_OTSU);
   cv::threshold( data.gray, data.binary,t*0.8,255,THRESH_BINARY);
   //if (g_showImageDebug) cv::imshow("OSTU", data.binary); 
	//cv::threshold(data.gray, data.binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // 5.面积阈值处理
    /*Img_areaThreshold(data.binary, data.binary, 100);
	if (g_showImageDebug) cv::imshow("last", data.binary);*/
    // 6.开操作
    Img_open(data.binary, data.binary, 1, 3);
    // 7.闭操作
    Img_close(data.binary, data.binary, 10, 5);
    // 8.细化骨架
    img_thinning zhangSuen;
    zhangSuen.zhangSuenThinning(data.binary, data.skeleton);
}

void ImgProcess::roipreprocessImage(ImgData& data) {
	//cv::imshow("original", data.inputcolor);
    // 1.双边滤波
    Img_bilateralFilter(data.input_roicolor, data.bilFilt_img, 3);
    // 2.灰度化
    cv::cvtColor(data.bilFilt_img, data.gray, cv::COLOR_BGR2GRAY);
	//cv::imshow("gray", data.gray);

	for (int i = 0; i < data.gray.cols; i++)
	{
		for (int j = 0; j < data.gray.rows; j++)
		{
			if (data.gray.at<uchar>(j, i) < 5)	//原本是7
			{
				data.gray.at<uchar>(j, i) = 0;
			}
		}
	}

	// 闭运算
	//Img_dilate(data.gray, data.gray, 1, 5);
	//cv::imshow("gray dilate", data.gray);
	//Img_close(data.gray, data.gray, 1, 5);
	//cv::imshow("gray close", data.gray);

    // 3.均值滤波
    cv::blur(data.gray, data.gray, cv::Size(5, 5));
    // 4.自适应阈值
    //Img_adaptiveThreshold(data.gray, data.binary);
	//cv::threshold(data.gray, data.binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
	double t = cv::threshold(data.gray, data.binary, 0, 255, THRESH_OTSU);
	cv::threshold(data.gray, data.binary, t*0.8, 255, THRESH_BINARY);
	
	//cv::imshow("gray ostu", data.binary);


	//Img_open(data.binary, data.binary, 1, 3);
	//cv::imshow("gray Median Blurred adaptiveThreshold open", data.binary);

    // 5.面积阈值处理
    Img_areaThreshold(data.binary, data.binary, 100);
    // 6.开操作
    //Img_open(data.binary, data.binary, 1, 3);
 //   // 7.闭操作
 //   Img_close(data.binary, data.binary, 10, 15);
	//cv::imshow("gray adaptiveThreshold TULV aera_thresh close", data.binary);
    // 8.细化骨架
    img_thinning zhangSuen;
    zhangSuen.zhangSuenThinning(data.binary, data.skeleton);
    data.global_skeleton = cv::Mat::zeros(data.inputcolor.size(), CV_8UC1);
    // Guard: only paste when a valid in-frame ROI exists; otherwise global_skeleton
    // stays all-zero and downstream detection degrades to the Hough fallback.
    if (!data.roi_region.empty()) {
        const cv::Rect r = data.roi_region.back();
        if (r.x >= 0 && r.y >= 0 && r.width > 0 && r.height > 0 &&
            r.x + r.width  <= data.global_skeleton.cols &&
            r.y + r.height <= data.global_skeleton.rows) {
            data.skeleton.copyTo(data.global_skeleton(r));
        }
    }
}

void ImgProcess::copySmallImg(cv::Mat& src, cv::Mat& dst, int x, int y) {};
void ImgProcess::copySmallImg(cv::Mat& src, cv::Mat& dst, Size size, cv::Rect roi) {
    dst = cv::Mat::zeros(size, CV_8UC1);
    src.copyTo(dst(roi));
};