#include "img_lines.h"

img_lines::img_lines(){}
img_lines::~img_lines(){}


// 检测长直线
bool img_lines::detectLongLines(Mat& skeleton, Mat& inputImg, vector<Vec4i>& long_lines, int threshold = 50, int minLineLength = 50, int maxLineGap = 100) {
    // HoughLinesP(skeleton, lines, 1, CV_PI / 180, 50, 20, 10);
    // HoughLinesP(skeleton, lines, 1, CV_PI / 180, 50, 50, 100);
	cv::Mat houghMat;
	HoughLinesP(skeleton, houghMat, 1, CV_PI / 180, threshold, minLineLength, maxLineGap);
	long_lines.clear();
	if (!houghMat.empty()) {
		const size_t houghCount = houghMat.total();
		const Vec4i* houghPtr = houghMat.ptr<Vec4i>();
		long_lines.reserve(houghCount);
		for (size_t i = 0; i < houghCount; ++i)
			long_lines.push_back(houghPtr[i]);
	}
    cout << "检测到" << long_lines.size() << "条直线" << endl;
    bool found_lines = false;
    if(long_lines.size() > 2){
        found_lines = true;
    }

    //输出lines里的所有直线方程
    for(Vec4i l : long_lines)
    {
        LineEquation eq = getLineEquation(l);
        if(eq.is_vertical == true){
            cout << "line: x = " << eq.x0 << endl;
        }
        else{
            cout << "line: y = " << eq.a << "x + " << eq.b << endl;
        }
    }

    // 新增绘制功能 ==========
    Mat lines_img = inputImg.clone();
    for (auto& l : long_lines) { // auto关键字自动推导变量类型，&表示引用
        line(lines_img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1);
    }
    if (g_showImageDebug) imshow("Long Lines", lines_img);
    // ======================
    return found_lines;
}

/**
 * @brief 从排序后的直线列表中寻找目标直线（垂直直线和倾斜直线）
 * 
 * @param long_lines 已按长度降序排序的直线列表
 * @param[out] vertical_line 输出的垂直直线
 * @param[out] incline_line 输出的倾斜直线
 * @return bool 是否成功找到两条目标直线
 */
bool img_lines::findTargetLines(vector<Vec4i>& long_lines, double angle, Vec4i& line) {
    if (long_lines.size() < 2) {
        cout << "检测到的直线不足2条,尝试下一帧..." << endl;
        return false;
    }

    // 按长度排序直线（从长到短）
    sort(long_lines.begin(), long_lines.end(),
        [](const Vec4i& a, const Vec4i& b) {
            double len_a = norm(Point(a[0], a[1]) - Point(a[2], a[3]));
            double len_b = norm(Point(b[0], b[1]) - Point(b[2], b[3]));
            return len_a > len_b; // 降序排序
        });

    // 寻找角度接近目标angle的直线
    double line_angle = angle;
    bool found_line = false;
    const double angle_min = line_angle - 5.0;
    const double angle_max = line_angle + 5.0;

    // 记录垂直直线的索引
    size_t line_index = -1;

    // 在已排序的长直线列表中寻找接近垂直的直线
    for (size_t i = 0; i < long_lines.size(); ++i) {
        LineEquation eq_temp = getLineEquation(long_lines[i]);
        double angle = eq_temp.is_vertical ? 90.0 : atan(eq_temp.a) * 180 / CV_PI;

        // 检查是否在目标角度范围内
        if (angle >= angle_min && angle <= angle_max) {
            line = long_lines[i];
            found_line = true;
            line_index = i; // 记录索引
            cout << "找到接近目标angle的直线: " << angle << "度" << endl;
            break;
        }
    }

    if (!found_line) {
        cout << "找不到接近目标angle的直线，结束处理..." << endl;
        return false;
    }

    return found_line;
}

// 在 long_lines 中寻找角度接近 angle 的直线，按"到图像原点的距离"降序输出，返回实际输出的条数
//   outLines[0] = 距离最大的直线（最远），outLines[1] = 距离最小的直线（最近）
int img_lines::findTargetLinesByAngle(vector<Vec4i>& long_lines, double angle, vector<Vec4i>& outLines, int count) {
    outLines.clear();
    if (long_lines.size() < 2 || count <= 0) {
        return 0;
    }

    // 1. 按长度降序排序，长直线优先参与角度筛选
    sort(long_lines.begin(), long_lines.end(),
        [](const Vec4i& a, const Vec4i& b) {
            double len_a = norm(Point(a[0], a[1]) - Point(a[2], a[3]));
            double len_b = norm(Point(b[0], b[1]) - Point(b[2], b[3]));
            return len_a > len_b;
        });

    // 2. 收集角度符合的直线，同时计算每条直线到图像原点 (0,0) 的距离
    const double angle_min = angle - 5.0;
    const double angle_max = angle + 5.0;
    Point origin(0, 0);

    struct Candidate {
        Vec4i  line;
        double dist;    // 直线到图像原点的距离
    };
    vector<Candidate> candidates;

    for (size_t i = 0; i < long_lines.size(); ++i) {
        LineEquation eq = getLineEquation(long_lines[i]);
        double line_angle = eq.is_vertical ? 90.0 : atan(eq.a) * 180.0 / CV_PI;
        if (line_angle < angle_min || line_angle > angle_max) {
            continue;
        }
        Candidate c;
        c.line = long_lines[i];
        c.dist = pointToLineDistance(origin, eq);
        candidates.push_back(c);
        cout << "候选直线: " << line_angle << " 度, 到原点距离 " << c.dist << endl;
    }

    if (candidates.empty()) {
        cout << "没有角度在 " << angle_min << "~" << angle_max << " 度范围内的直线" << endl;
        return 0;
    }

    // 3. 按到原点的距离降序排序：远的在前，近的在后
    sort(candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) {
            return a.dist > b.dist;
        });

    // 4. 输出：count == 2 且候选多于 2 条时取"最远 + 最近"；其余情况按距离降序取前 count 条
    if (count == 2 && candidates.size() > 2) {
        outLines.push_back(candidates.front().line);    // [0] 距离最大
        outLines.push_back(candidates.back().line);     // [1] 距离最小
    }
    else {
        const int n = min((int)candidates.size(), count);
        for (int i = 0; i < n; ++i) {
            outLines.push_back(candidates[i].line);
        }
    }

    cout << "近垂直线按到原点距离排序: 最远 " << candidates.front().dist
         << ", 最近 " << candidates.back().dist << ", 输出 " << outLines.size() << " 条" << endl;
    return (int)outLines.size();
}


// 新增：计算点到直线方程的距离
double img_lines::pointToLineDistance(Point& p, LineEquation& eq) {
    if (eq.is_vertical) {
        return abs(p.x - eq.x0);
    }
    return abs(eq.a * p.x - p.y + eq.b) / sqrt(eq.a * eq.a + 1);
}

// RANSAC拟合直线,upper_line和lower_line为输出,是以点表示直线，不是直线方程。
void img_lines::RANSAC_fitLine(cv::Mat& upper_roi, cv::Mat& lower_roi, Vec4f& upper_line, Vec4f& lower_line){
    // 提取上区域的点
    vector<Point> upper_points;
    for (int y = 0; y < upper_roi.rows; ++y) {
        for (int x = 0; x < upper_roi.cols; ++x) {
            if (upper_roi.at<uchar>(y, x) != 0) {
                upper_points.push_back(Point(x, y));
            }
        }
    }

    if (!upper_points.empty()) {
        // 使用RANSAC进行初始拟合
        cv::fitLine(upper_points, upper_line, cv::DIST_L2, 0, 0.01, 0.01);
        //cout << "上区域拟合的直线方程为: " << upper_line << endl;

        // 计算点到直线的距离
        LineEquation eq = getLineEquation(upper_line);
        vector<double> distances;
        for (auto& p : upper_points) {
            distances.push_back(pointToLineDistance(p, eq));
        }

        // 计算中位数绝对偏差(MAD)
        vector<double> sorted_distances = distances;
        sort(sorted_distances.begin(), sorted_distances.end());
        double median_distance = sorted_distances[distances.size() / 2];

        vector<double> abs_deviations;
        for (double d : distances) {
            abs_deviations.push_back(fabs(d - median_distance));
        }
        sort(abs_deviations.begin(), abs_deviations.end());
        double mad = abs_deviations[abs_deviations.size() / 2];

        // 设定阈值（使用3倍MAD）
        const double threshold = median_distance + 3 * 1.4826 * mad; // 1.4826是正态分布常数

        // 过滤离群点
        vector<Point> filtered_upper_points;
        for (size_t i = 0; i < upper_points.size(); ++i) {
            if (distances[i] < threshold) {
                filtered_upper_points.push_back(upper_points[i]);
            }
        }

        // 使用过滤后的点重新拟合
        if (!filtered_upper_points.empty() && filtered_upper_points.size() >= 2) {
            cv::fitLine(filtered_upper_points, upper_line, cv::DIST_L2, 0, 0.01, 0.01);
            // // 更新lineEquation_vertical
            // lineEquation_vertical = getLineEquation(upper_line); 
        }
    }


    // 提取下区域的点
    vector<Point> lower_points;
    for (int y = 0; y < lower_roi.rows; ++y) {
        for (int x = 0; x < lower_roi.cols; ++x) {
            if (lower_roi.at<uchar>(y, x) != 0) {
                lower_points.push_back(Point(x, y));
            }
        }
    }

    if (!lower_points.empty()) {
        // 使用RANSAC进行初始拟合
        cv::fitLine(lower_points, lower_line, cv::DIST_L2, 0, 0.01, 0.01);
        //cout << "下区域拟合的直线方程为: " << lower_line << endl;
        //cv::waitKey(0);

        // 计算点到直线的距离
        LineEquation eq = getLineEquation(lower_line);
        vector<double> distances;
        for (auto& p : lower_points) {
            distances.push_back(pointToLineDistance(p, eq));
        }

        // 计算中位数绝对偏差(MAD)
        vector<double> sorted_distances = distances;
        sort(sorted_distances.begin(), sorted_distances.end());
        double median_distance = sorted_distances[distances.size() / 2];

        vector<double> abs_deviations;
        for (double d : distances) {
            abs_deviations.push_back(fabs(d - median_distance));
        }
        sort(abs_deviations.begin(), abs_deviations.end());
        double mad = abs_deviations[abs_deviations.size() / 2];

        // 设定阈值（使用3倍MAD）
        const double threshold = median_distance + 3 * 1.4826 * mad; // 1.4826是正态分布常数

        // 过滤离群点
        vector<Point> filtered_lower_points;
        for (size_t i = 0; i < lower_points.size(); ++i) {
            if (distances[i] < threshold) {
                filtered_lower_points.push_back(lower_points[i]);
            }
        }

        // 使用过滤后的点重新拟合
        if (!filtered_lower_points.empty() && filtered_lower_points.size() >= 2) {
            cv::fitLine(filtered_lower_points, lower_line, cv::DIST_L2, 0, 0.01, 0.01);
            //cout << "下区域拟合的直线方程为: " << lower_line << endl;
            // // 更新lineEquation_incline
            // lineEquation_incline = getLineEquation(lower_line);
        }
    }
}


// RANSAC拟合直线,line为输出,是以点表示直线，不是直线方程。
void img_lines::RANSAC_fitLine(cv::Mat& src, Vec4f& line) {
    // 提取上区域的点
    vector<Point> points;
    for (int y = 0; y < src.rows; ++y) {
        for (int x = 0; x < src.cols; ++x) {
            if (src.at<uchar>(y, x) != 0) {
                points.push_back(Point(x, y));
            }
        }
    }

    if (!points.empty()) {
        // 使用RANSAC进行初始拟合
        cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
        //cout << "上区域拟合的直线方程为: " << upper_line << endl;

        // 计算点到直线的距离
        LineEquation eq = getLineEquation(line);
        vector<double> distances;
        for (auto& p : points) {
            distances.push_back(pointToLineDistance(p, eq));
        }

        // 计算中位数绝对偏差(MAD)
        vector<double> sorted_distances = distances;
        sort(sorted_distances.begin(), sorted_distances.end());
        double median_distance = sorted_distances[distances.size() / 2];

        vector<double> abs_deviations;
        for (double d : distances) {
            abs_deviations.push_back(fabs(d - median_distance));
        }
        sort(abs_deviations.begin(), abs_deviations.end());
        double mad = abs_deviations[abs_deviations.size() / 2];

        // 设定阈值（使用3倍MAD）
        const double threshold = median_distance + 3 * 1.4826 * mad; // 1.4826是正态分布常数

        // 过滤离群点
        vector<Point> filtered_upper_points;
        for (size_t i = 0; i < points.size(); ++i) {
            if (distances[i] < threshold) {
                filtered_upper_points.push_back(points[i]);
            }
        }

        // 使用过滤后的点重新拟合
        if (!filtered_upper_points.empty() && filtered_upper_points.size() >= 2) {
            cv::fitLine(filtered_upper_points, line, cv::DIST_L2, 0, 0.01, 0.01);
        }
    }
}
