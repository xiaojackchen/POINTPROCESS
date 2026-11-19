#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "img_process.h"//已经引入img_data.h
#include "img_roi.h"

using namespace cv;
using namespace std;

class img_lines {
public:
    img_lines();
    ~img_lines();

public:
    // 检测长直线
    bool detectLongLines(cv::Mat& skeleton, cv::Mat& inputImg, vector<Vec4i>& long_lines, int threshold, int minLineLength, int maxLineGap);
    
    // 输入 Vec4i 表示线段端点坐标：(x1, y1, x2, y2)（来自线段检测算法如 HoughLinesP）
    template<typename T>//作用域仅限于紧随其后的单个函数或类、结构体
    LineEquation getLineEquation(T& line) { // 等价于const Vec4i* const line;常量引用line的值无法修改
        Point p1(line[0], line[1]);
        Point p2(line[2], line[3]);

        LineEquation eq;
        if (p2.x == p1.x) { // 处理垂直线
            eq.is_vertical = true;
            eq.x0 = p1.x;
        }
        else {
            eq.a = double(p2.y - p1.y) / (p2.x - p1.x);
            eq.b = p1.y - eq.a * p1.x;
        }
        return eq;
    }

    //输入 Vec4f 表示拟合直线参数：(vx, vy, x0, y0)（来自 cv::fitLine 最小二乘拟合结果）
    template<typename T>
    LineEquation fitLineToEquation(T& lineParams) {
        LineEquation eq;
        float vx = lineParams[0], vy = lineParams[1];
        float x0 = lineParams[2], y0 = lineParams[3];

        if (fabs(vx) < 1e-5) { // 处理垂直线
            eq.is_vertical = true;
            eq.x0 = x0;
        }
        else {
            eq.a = vy / vx;      // 计算斜率
            eq.b = y0 - eq.a * x0; // 计算截距
            eq.is_vertical = false;
        }
        return eq;
    }

    // 寻找接近90°和-65°的直线
    bool findTargetLines(vector<Vec4i>& long_lines, double angle, Vec4i& line);
    /* 寻找角度接近 angle 的直线，按“到图像原点的距离”降序输出，返回实际输出的条数：
       outLines[0] = 距离最大的直线（最远），outLines[1] = 距离最小的直线（最近）；
       count == 2 且候选多于 2 条时，直接取最远和最近两条；
       角度筛选前先按长度降序排序，长直线优先。 */
    int findTargetLinesByAngle(vector<Vec4i>& long_lines, double angle, vector<Vec4i>& outLines, int count = 2);

    // 新增：计算点到直线方程的距离
    double pointToLineDistance(Point& p, LineEquation& eq);

    /*计算交点，用于经过Hough拟合得到的直线的交点计算。
     参数：vertical_line: 垂直线，incline_line: 斜线是Hough变换得到的结果，
     参数：lineEquation_vertical: 垂直线方程，lineEquation_incline: 斜线方程。是LineEquation结构形式
   */
    template<typename T>
    bool calculateIntersectionHough(T& vertical_line, T& incline_line, Point2d& intersection, LineEquation& lineEquation_vertical, LineEquation& lineEquation_incline) {
        lineEquation_vertical = getLineEquation(vertical_line);
        cout << "vertical_line: " << lineEquation_vertical.a << "x + " << lineEquation_vertical.b << endl;
        lineEquation_incline = getLineEquation(incline_line);
        cout << "incline_line: " << lineEquation_incline.a << "x + " << lineEquation_incline.b << endl;
        // 处理两垂直线情况
        if (lineEquation_vertical.is_vertical && lineEquation_incline.is_vertical) {
            return false; // 平行不相交
        }

        // 处理一条垂直线情况
        if (lineEquation_vertical.is_vertical) {
            intersection.x = lineEquation_vertical.x0;
            intersection.y = lineEquation_incline.a * intersection.x + lineEquation_incline.b;
            return true;
        }
        if (lineEquation_incline.is_vertical) {
            intersection.x = lineEquation_incline.x0;
            intersection.y = lineEquation_vertical.a * intersection.x + lineEquation_vertical.b;
            return true;
        }

        // 一般情况 y = a1x + b1 和 y = a2x + b2
        if (fabs(lineEquation_vertical.a - lineEquation_incline.a) < 1e-5) {
            return false; // 平行线
        }

        intersection.x = (lineEquation_incline.b - lineEquation_vertical.b) / (lineEquation_vertical.a - lineEquation_incline.a);
        intersection.y = lineEquation_vertical.a * intersection.x + lineEquation_vertical.b;
        return true;
    }

    /*计算交点，用于经过Fitline拟合得到的直线的交点计算。
      参数：vertical_line: 垂直线，incline_line: 斜线是Fitline拟合得到的结果，
      参数：lineEquation_vertical: 垂直线方程，lineEquation_incline: 斜线方程。是LineEquation结构形式
    */ 
    template<typename T>
    bool calculateIntersectionFitline(T& vertical_line, T& incline_line, Point2d& intersection, LineEquation& lineEquation_vertical, LineEquation& lineEquation_incline) {
        lineEquation_vertical = fitLineToEquation(vertical_line);
        cout << "vertical_line: " << lineEquation_vertical.a << "x + " << lineEquation_vertical.b << endl;
        lineEquation_incline = fitLineToEquation(incline_line);
        cout << "incline_line: " << lineEquation_incline.a << "x + " << lineEquation_incline.b << endl;
        // 处理两垂直线情况
        if (lineEquation_vertical.is_vertical && lineEquation_incline.is_vertical) {
            return false; // 平行不相交
        }

        // 处理一条垂直线情况
        if (lineEquation_vertical.is_vertical) {
            intersection.x = lineEquation_vertical.x0;
            intersection.y = lineEquation_incline.a * intersection.x + lineEquation_incline.b;
            return true;
        }
        if (lineEquation_incline.is_vertical) {
            intersection.x = lineEquation_incline.x0;
            intersection.y = lineEquation_vertical.a * intersection.x + lineEquation_vertical.b;
            return true;
        }

        // 一般情况 y = a1x + b1 和 y = a2x + b2
        if (fabs(lineEquation_vertical.a - lineEquation_incline.a) < 1e-5) {
            return false; // 平行线
        }

        intersection.x = (lineEquation_incline.b - lineEquation_vertical.b) / (lineEquation_vertical.a - lineEquation_incline.a);
        intersection.y = lineEquation_vertical.a * intersection.x + lineEquation_vertical.b;
        return true;
    }

    // 保存交点并可视化，用于经过Hough变换处理检测到的直线与交点可视化
    template<typename T>
    void drawHoughPoint(cv::Mat& inputImg, Point2d& intersection, T vertical_line, T incline_line, std::vector<cv::Point2d>& current_interPoint)
    {
        // 9. 保存交点并可视化
		if(intersection.x>0 && intersection.y>0 && intersection.x<inputImg.cols && intersection.y<inputImg.rows){
			current_interPoint.push_back(intersection);
			cout << "成功检测到交点: (" << intersection.x << ", " << intersection.y << ")" << endl;

			// 可视化结果
			Mat resultImg = inputImg.clone();
			line(resultImg, Point(vertical_line[0], vertical_line[1]), Point(vertical_line[2], vertical_line[3]), Scalar(0, 255, 0), 1);
			line(resultImg, Point(incline_line[0], incline_line[1]), Point(incline_line[2], incline_line[3]), Scalar(255, 0, 0), 1);

			circle(resultImg, intersection, 3, Scalar(0, 255, 255), FILLED);
			putText(resultImg, "Intersection Point", Point(intersection.x + 10.0, intersection.y - 10.0), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);

			if (g_showImageDebug) imshow("result", resultImg);
		}
    }

    // 保存交点并可视化，用于经过Fitline拟合检测到的直线与交点可视化
    template<typename T>
    void drawFitlinePoint(cv::Mat& inputImg, Point2d& intersection, T vertical_line, T incline_line, std::vector<cv::Point2d>& current_interPoint)
    {
        // 9. 保存交点并可视化
		if (intersection.x > 0 && intersection.y > 0 && intersection.x < inputImg.cols && intersection.y < inputImg.rows) {
			current_interPoint.push_back(intersection);
			cout << "成功检测到交点: (" << intersection.x << ", " << intersection.y << ")" << endl;

			// 可视化结果
			Mat resultImg = inputImg.clone();

			float vx_upper = vertical_line[0], vy_upper = vertical_line[1];
			float x0_upper = vertical_line[2], y0_upper = vertical_line[3];
			Point pt1_upper(static_cast<int>(x0_upper - vx_upper * 1000.), static_cast<int>(y0_upper - vy_upper * 1000.));
			Point pt2_upper(static_cast<int>(x0_upper + vx_upper * 1000.), static_cast<int>(y0_upper + vy_upper * 1000.));
			line(resultImg, pt1_upper, pt2_upper, Scalar(0, 255, 0), 1);
			float vx_lower = incline_line[0], vy_lower = incline_line[1];
			float x0_lower = incline_line[2], y0_lower = incline_line[3];
			Point pt1_lower(static_cast<int>(x0_lower - vx_lower * 1000.), static_cast<int>(y0_lower - vy_lower * 1000.));
			Point pt2_lower(static_cast<int>(x0_lower + vx_lower * 1000.), static_cast<int>(y0_lower + vy_lower * 1000.));
			line(resultImg, pt1_lower, pt2_lower, Scalar(255, 0, 0), 1);

			circle(resultImg, intersection, 3, Scalar(0, 255, 255), FILLED);
			putText(resultImg, "Intersection Point", Point(intersection.x + 10, intersection.y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
			putText(resultImg,"(" + std::to_string(intersection.x) + "," + std::to_string(intersection.y) + ")" , Point(intersection.x + 10, intersection.y + 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
			if (g_showImageDebug) imshow("result", resultImg);
		}
    }

    // RANSAC拟合直线
    void RANSAC_fitLine(cv::Mat& upper_roi, cv::Mat& lower_roi, Vec4f& upper_line, Vec4f& lower_line);
    void RANSAC_fitLine(cv::Mat& src, Vec4f& line);
};
