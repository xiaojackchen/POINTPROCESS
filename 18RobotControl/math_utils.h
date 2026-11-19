#pragma once
#ifndef MATH_UTILS_H
#define MATH_UTILS_H
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

enum Trend {
	T_INCREASE,      // 严格单调增
	T_DECREASE,      // 严格单调减
	T_UP_DOWN,       // 先增后减（∧）
	T_DOWN_UP,       // 先减后增（∨）
	T_OTHER          // 多次拐点 / 平坦
};

Trend judgeTrend(const vector<double>& v);

int simpleTrendDetection1(const std::vector<double>& data);

int simpleTrendDetection(const std::vector<double>& data, const std::vector<double>& data1);

int analyzeTrend(const std::vector<double>& data);

struct coeffs {
	double a;
	double b;
	double c;
	double index;
	double value;
};

// 二次函数拟合和极值点求解
vector<coeffs> quadraticFitAndExtremes(const vector<Point2d>& points);
// RANSAC二次曲线拟合
vector<coeffs> ransacQuadraticFit(const vector<Point2d>& points, int max_iterations, double threshold, double min_inlier_ratio);
std::vector<double> removeOutliersIQRSmallSample(const std::vector<double>& data, double multiplier = 1.5);
void plotSequence(const std::vector<cv::Point2d>& points, const coeffs& coeff, char which, int W = 800, int H = 600);
// 把 double 值线性映射到图像行列
cv::Point2d mapToImg(double x, double y, double xMin, double xMax, double yMin, double yMax, int W, int H);
std::pair<double, double> LinearFit(const std::vector<double>& xData, const std::vector<double>& yData, char which);

// 线性拟合绘图函数
void plotLinearFit(const std::vector<double>& xData, const std::vector<double>& yData,
	const std::pair<double, double>& coefficients,
	const std::string& title = "Linear Fit",
	int W = 800, int H = 600);

#endif // MATH_UTILS_H
