#include "math_utils.h"
#include <numeric>
#include <random>


int simpleTrendDetection1(const std::vector<double>& data)
{
	int n = data.size();
	if (n < 5) return 5;

	if (data[n - 1] > data[n - 2] && data[n - 2] > data[n - 3]) 
		return 1;// 单增
	if (data[n - 1] > data[n - 2] && data[n - 2] < data[n - 3]) 
		return 4; // 先减后增
	if (data[n - 1] < data[n - 2] && data[n - 2] > data[n - 3]) 
		return 3; // 先增后减
	if (data[n - 1] < data[n - 2] && data[n - 2] < data[n - 3]) 
		return 2; // 单减

	return 5; // 其他
}

int simpleTrendDetection(const std::vector<double>& data, const std::vector<double>& data1) {
	if (data.size() < 5) return 5;

	// 只分析首尾各25%的数据
	int n = data.size();
	int sample_size = std::max(3, n / 4);

	// 计算首段和尾段的平均斜率
	double first_slope = 0, last_slope = 0;

	// 首段斜率（前sample_size个点）
	for (int i = 1; i < sample_size; i++) {
		first_slope += (data[i] - data[i - 1]);
	}
	first_slope /= (sample_size - 1);

	// 尾段斜率（后sample_size个点）
	for (int i = n - sample_size + 1; i < n; i++) {
		last_slope += (data[i] - data[i - 1]);
	}
	last_slope /= (sample_size - 1);

	// 简单阈值
	double threshold = 0.0001;

	if (first_slope < -threshold && last_slope > threshold) 
		return 4; // 先减后增
	if (first_slope > threshold && last_slope < -threshold)
	{
		//if (data1.begin() > data1.end())//
		//	return 2;	// 单减//
		//else//
			return 3; // 先增后减
	}
		
	if ((first_slope > threshold && last_slope > threshold))
	{
		//if (data1.begin() > data1.end())//
		//{//
		//	return 2; // 单减//
		//}//
		//else//
			return 1; // 单增
	}
		
	if (first_slope < -threshold && last_slope < -threshold) 
		return 2; // 单减

	return 5; // 其他
}
//1、单增
//2、单减
//3、先增后减
//4、先减后增
//5、其他
int analyzeTrend(const std::vector<double>& data) {
	if (data.size() < 5) return 5; // 数据不足返回"其他"

	int n = data.size();
	int seg1_end = n / 3;
	int seg2_end = 2 * n / 3;

	// 计算三段斜率
	auto slope = [](const std::vector<double>& seg) {
		int m = seg.size();
		double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
		for (int i = 0; i < m; i++) {
			sum_x += i;
			sum_y += seg[i];
			sum_xy += i * seg[i];
			sum_xx += i * i;
		}
		return (m * sum_xy - sum_x * sum_y) / (m * sum_xx - sum_x * sum_x);
	};

	std::vector<double> seg1(data.begin(), data.begin() + seg1_end);
	std::vector<double> seg2(data.begin() + seg1_end, data.begin() + seg2_end);
	std::vector<double> seg3(data.begin() + seg2_end, data.end());

	double s1 = slope(seg1);
	double s2 = slope(seg2);
	double s3 = slope(seg3);
	double overall = slope(data);

	double threshold = 0.001 * (*std::max_element(data.begin(), data.end()) -
		*std::min_element(data.begin(), data.end()));

	// 核心判断逻辑
	if (s1 > threshold && s2 > threshold && s3 > threshold) return 1; // 单增
	if (s1 < -threshold && s2 < -threshold && s3 < -threshold) return 2; // 单减
	if ((s1 > threshold && s3 < -threshold) || (s1 > threshold && s2 < -threshold)) return 3; // 先增后减
	if ((s1 < -threshold && s3 > threshold) || (s1 < -threshold && s2 > threshold)) return 4; // 先减后增

																							  // 基于整体趋势的最后判断
	if (overall > threshold) return 1; // 单增
	if (overall < -threshold) return 2; // 单减

	return 5; // 其他
}

Trend judgeTrend(const vector<double>& v)
{
	if (v.size() < 3) return T_OTHER;

	int up = 0, down = 0, flat = 0;
	int turn = 0;                // 拐点次数
	int lastDir = 0;             // 1 up  -1 down  0 flat

	for (size_t i = 1; i < v.size(); ++i)
	{
		double diff = v[i] - v[i - 1];
		if (fabs(diff) < 1e-12) { flat++; continue; } // 忽略平坦
		int dir = diff > 0 ? 1 : -1;
		if (lastDir != 0 && lastDir != dir) turn++;   // 方向变化→拐点
		lastDir = dir;
		if (dir == 1) up++;
		else down++;
	}

	if (turn == 0) {
		if (up > 0) return T_INCREASE;
		if (down > 0) return T_DECREASE;
		return T_OTHER;          // 全 flat
	}
	if (turn == 1) {
		if (lastDir == -1) return T_UP_DOWN;   // 结束时下降→∧
		else return T_DOWN_UP;                 // 结束时上升→∨
	}
	return T_OTHER;              // 多次拐点
}

// 辅助：把 double 值线性映射到图像行列
cv::Point2d mapToImg(double x, double y, double xMin, double xMax, double yMin, double yMax, int W, int H)
{
	int u = static_cast<int>((x - xMin) / (xMax - xMin) * (W - 1) + 0.5);
	int v = static_cast<int>((yMax - y) / (yMax - yMin) * (H - 1) + 0.5);
	return cv::Point2d(u, v);
}

void plotSequence(const std::vector<cv::Point2d>& points, const coeffs& coeff, char which, int W, int H)
{
	if (points.empty()) return;

	/* 1. 准备数据：根据 which 决定用 x 还是 y */
	std::vector<double> idx(points.size()), val(points.size());
	for (size_t i = 0; i < points.size(); ++i) {
		idx[i] = static_cast<double>(i);
		val[i] = (which == 'x' ? points[i].x : points[i].y);
	}

	double a = coeff.a;
	double b = coeff.b;
	double c = coeff.c;

	/* 2. 坐标范围（完全保持原样）*/
	auto mmX = std::minmax_element(idx.begin(), idx.end());
	auto mmY = std::minmax_element(val.begin(), val.end());
	double dx = *mmX.second - *mmX.first;
	double dy = *mmY.second - *mmY.first;
	double xMin = *mmX.first - 0.05 * dx;
	double xMax = *mmX.second + 0.05 * dx;
	double yMin = *mmY.first - 0.05 * dy;
	double yMax = *mmY.second + 0.05 * dy;

	/* 3. 画布（保持原样）*/
	cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

	/* 4. 坐标轴（保持原样）*/
	cv::line(img, cv::Point(40, H - 40), cv::Point(W - 20, H - 40), cv::Scalar(0, 0, 0), 2);
	cv::line(img, cv::Point(40, 20), cv::Point(40, H - 40), cv::Scalar(0, 0, 0), 2);

	/* 新增：绘制坐标轴刻度线 */
	const int numTicks = 5; // 每个轴的刻度数量
	const int tickLength = 5; // 刻度线长度

							  // X轴刻度
	for (int i = 0; i <= numTicks; ++i) {
		double xValue = xMin + i * (xMax - xMin) / numTicks;
		cv::Point2d tickPos = mapToImg(xValue, yMin, xMin, xMax, yMin, yMax, W, H);

		// 绘制刻度线
		cv::line(img,
			cv::Point(tickPos.x, H - 40),
			cv::Point(tickPos.x, H - 40 + tickLength),
			cv::Scalar(0, 0, 0), 1);

		// 绘制刻度值
		cv::putText(img,
			cv::format("%.2f", xValue),
			cv::Point(tickPos.x - 15, H - 20),
			cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
	}

	// Y轴刻度
	for (int i = 0; i <= numTicks; ++i) {
		double yValue = yMin + i * (yMax - yMin) / numTicks;
		cv::Point2d tickPos = mapToImg(xMin, yValue, xMin, xMax, yMin, yMax, W, H);

		// 绘制刻度线
		cv::line(img,
			cv::Point(40, tickPos.y),
			cv::Point(40 - tickLength, tickPos.y),
			cv::Scalar(0, 0, 0), 1);

		// 绘制刻度值
		cv::putText(img,
			cv::format("%.2f", yValue),
			cv::Point(10, tickPos.y + 5),
			cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
	}

	/* 5. 散点（保持原样：红色）*/
	for (size_t i = 0; i < idx.size(); ++i) {
		cv::Point2d p = mapToImg(idx[i], val[i], xMin, xMax, yMin, yMax, W, H);
		cv::circle(img, p, 4, cv::Scalar(0, 0, 255), -1);
	}

	/* 6. 拟合曲线（保持原样：蓝色）*/
	const int NCurve = 200;
	for (int k = 0; k < NCurve - 1; ++k) {
		double t0 = xMin + (xMax - xMin) * k / (NCurve - 1.0);
		double t1 = xMin + (xMax - xMin) * (k + 1) / (NCurve - 1.0);
		double y0 = a * t0 * t0 + b * t0 + c;
		double y1 = a * t1 * t1 + b * t1 + c;
		cv::Point2d p0 = mapToImg(t0, y0, xMin, xMax, yMin, yMax, W, H);
		cv::Point2d p1 = mapToImg(t1, y1, xMin, xMax, yMin, yMax, W, H);
		cv::line(img, p0, p1, cv::Scalar(255, 0, 0), 2);
	}

	/* 7. 极值点（保持原样：绿色圆点）*/
	cv::Point2d p = mapToImg(coeff.index, coeff.value, xMin, xMax, yMin, yMax, W, H);
	cv::circle(img, p, 4, cv::Scalar(0, 255, 0), -1);
	cv::putText(img,
		cv::format("Pole: (%.2f, %.2f)", coeff.index, coeff.value),
		cv::Point2d(p.x - 60, p.y - 40),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);

	/* 8. 轴标签与标题：把 x 动态换成 y */
	cv::putText(img, "index", cv::Point(W - 60, H - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	cv::putText(img, string(1, which), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	cv::putText(img,
		cv::format("fit: %c=%f*i^2+%f*i+%f", which, a, b, c),
		cv::Point(50, 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);

	/* 9. 显示 */
	//cv::imshow(cv::format("%c sequence and quadratic fit", which), img);
}

// RANSAC二次曲线拟合
vector<coeffs> ransacQuadraticFit(const vector<Point2d>& points, int max_iterations,
								  double threshold, double min_inlier_ratio) 
{
	if (points.size() < 3) {
		cout << "数据点不足，使用普通最小二乘拟合" << endl;
		return quadraticFitAndExtremes(points);
	}

	vector<double> indices;
	for (size_t i = 0; i < points.size(); ++i) {
		indices.push_back(static_cast<double>(i));
	}

	// RANSAC拟合函数
	auto ransacFit = [&](const vector<double>& values) -> coeffs {
		coeffs best_model;
		int best_inliers = 0;
		vector<int> best_inlier_indices;

		// 生成所有可能的索引
		vector<int> all_indices(points.size());
		iota(all_indices.begin(), all_indices.end(), 0);

		random_device rd;
		mt19937 g(rd());

		for (int iter = 0; iter < max_iterations; ++iter) {
			// 随机选择3个不重复的点
			shuffle(all_indices.begin(), all_indices.end(), g);
			vector<int> sample_indices(all_indices.begin(), all_indices.begin() + 3);

			// 构建矩阵方程 A * coeffs = b
			Mat A(3, 3, CV_64F);
			Mat b(3, 1, CV_64F);

			for (int i = 0; i < 3; ++i) {
				int idx = sample_indices[i];
				double t = indices[idx];
				A.at<double>(i, 0) = t * t;
				A.at<double>(i, 1) = t;
				A.at<double>(i, 2) = 1.0;
				b.at<double>(i, 0) = values[idx];
			}

			Mat coeffs_mat;
			if (solve(A, b, coeffs_mat, DECOMP_SVD)) {
				double a = coeffs_mat.at<double>(0, 0);
				double b_val = coeffs_mat.at<double>(1, 0);
				double c = coeffs_mat.at<double>(2, 0);

				// 统计内点
				vector<int> inlier_indices;
				for (size_t i = 0; i < values.size(); ++i) {
					double predicted = a * indices[i] * indices[i] + b_val * indices[i] + c;
					if (abs(predicted - values[i]) <= threshold) {
						inlier_indices.push_back(i);
					}
				}

				if (inlier_indices.size() > best_inliers) {
					best_inliers = inlier_indices.size();
					best_inlier_indices = inlier_indices;
					best_model.a = a;
					best_model.b = b_val;
					best_model.c = c;
				}

				// 提前终止：如果找到足够多的内点
				if (best_inliers >= points.size() * min_inlier_ratio && iter > 50) {
					break;
				}
			}
		}

		// 用所有内点重新拟合最终模型
		if (best_inlier_indices.size() >= 3) {
			Mat A_final(best_inlier_indices.size(), 3, CV_64F);
			Mat b_final(best_inlier_indices.size(), 1, CV_64F);

			for (size_t i = 0; i < best_inlier_indices.size(); ++i) {
				int idx = best_inlier_indices[i];
				double t = indices[idx];
				A_final.at<double>(i, 0) = t * t;
				A_final.at<double>(i, 1) = t;
				A_final.at<double>(i, 2) = 1.0;
				b_final.at<double>(i, 0) = values[idx];
			}

			Mat final_coeffs;
			if (solve(A_final, b_final, final_coeffs, DECOMP_SVD)) {
				best_model.a = final_coeffs.at<double>(0, 0);
				best_model.b = final_coeffs.at<double>(1, 0);
				best_model.c = final_coeffs.at<double>(2, 0);
			}
		}

		// 计算极值点
		if (abs(best_model.a) > 1e-10) {
			best_model.index = -best_model.b / (2 * best_model.a);
			best_model.value = best_model.a * best_model.index * best_model.index +
				best_model.b * best_model.index + best_model.c;
		}
		else {
			best_model.index = 0;
			best_model.value = 0;
		}

		cout << "RANSAC找到 " << best_inliers << " 个内点（共 " << points.size() << " 个点）" << endl;
		return best_model;
	};

	// 分别对x和y坐标进行RANSAC拟合
	vector<double> x_values, y_values;
	for (const auto& p : points) {
		x_values.push_back(p.x);
		y_values.push_back(p.y);
	}

	coeffs x_model = ransacFit(x_values);
	coeffs y_model = ransacFit(y_values);

	vector<coeffs> result = { x_model, y_model };
	return result;
}

// 二次函数拟合和极值点求解
vector<coeffs> quadraticFitAndExtremes(const vector<Point2d>& points) {
	if (points.empty()) {
		cout << "错误: 数据点为空!" << endl;
		return{};
	}

	// 准备数据：索引序列和对应的x,y值
	vector<double> indices, x_data, y_data;
	for (size_t i = 0; i < points.size(); ++i) {
		indices.push_back(static_cast<double>(i));
		x_data.push_back(points[i].x);
		y_data.push_back(points[i].y);
	}

	// 二次函数拟合x坐标：x(i) = a*i? + b*i + c
	Mat A(indices.size(), 3, CV_64F);
	Mat b_x(x_data.size(), 1, CV_64F);
	Mat b_y(y_data.size(), 1, CV_64F);

	for (size_t i = 0; i < indices.size(); ++i) {
		double idx = indices[i];
		A.at<double>(i, 0) = idx * idx;  // i?
		A.at<double>(i, 1) = idx;        // i
		A.at<double>(i, 2) = 1.0;        // 常数项

		b_x.at<double>(i, 0) = x_data[i];
		b_y.at<double>(i, 0) = y_data[i];
	}

	// 求解拟合系数
	Mat coeffs_x, coeffs_y;
	solve(A, b_x, coeffs_x, DECOMP_SVD);
	solve(A, b_y, coeffs_y, DECOMP_SVD);

	double a_x = coeffs_x.at<double>(0, 0);
	double b_x_val = coeffs_x.at<double>(1, 0);
	double c_x = coeffs_x.at<double>(2, 0);

	double a_y = coeffs_y.at<double>(0, 0);
	double b_y_val = coeffs_y.at<double>(1, 0);
	double c_y = coeffs_y.at<double>(2, 0);

	// 输出拟合结果
	cout << "=== X坐标二次函数拟合结果 ===" << endl;
	cout << "拟合方程: x(i) = " << a_x << " * i? + " << b_x_val << " * i + " << c_x << endl;

	cout << "=== Y坐标二次函数拟合结果 ===" << endl;
	cout << "拟合方程: y(i) = " << a_y << " * i? + " << b_y_val << " * i + " << c_y << endl;

	// 计算二次函数的极值点
	// 对于二次函数 f(i) = a*i? + b*i + c，极值点出现在 i = -b/(2a)

	cout << "\n=== 极值点分析 ===" << endl;

	double extreme_i_x = 0.0;
	double extreme_value_x = 0.0;
	double extreme_i_y = 0.0;
	double extreme_value_y = 0.0;
	// X坐标极值点
	if (abs(a_x) > 1e-10) {  // 避免除零
		extreme_i_x = -b_x_val / (2 * a_x);
		extreme_value_x = a_x * extreme_i_x * extreme_i_x + b_x_val * extreme_i_x + c_x;

		cout << "X坐标极值点:" << endl;
		cout << "位置: i = " << extreme_i_x << endl;
		cout << "极值: x = " << extreme_value_x << endl;

		if (a_x > 0) {
			cout << "类型: 极小值 (因为二次项系数 " << a_x << " > 0)" << endl;
		}
		else {
			cout << "类型: 极大值 (因为二次项系数 " << a_x << " < 0)" << endl;
		}

		// 检查极值点是否在数据范围内
		if (extreme_i_x >= 0 && extreme_i_x <= indices.size() - 1) {
			cout << "极值点在数据范围内 [0, " << indices.size() - 1 << "]" << endl;
		}
		else {
			cout << "极值点不在数据范围内" << endl;
		}
	}
	else {
		cout << "X坐标拟合为线性函数，无极值点" << endl;
	}

	cout << endl;

	// Y坐标极值点
	if (abs(a_y) > 1e-10) {  // 避免除零
		extreme_i_y = -b_y_val / (2 * a_y);
		extreme_value_y = a_y * extreme_i_y * extreme_i_y + b_y_val * extreme_i_y + c_y;

		cout << "Y坐标极值点:" << endl;
		cout << "位置: i = " << extreme_i_y << endl;
		cout << "极值: y = " << extreme_value_y << endl;

		if (a_y > 0) {
			cout << "类型: 极小值 (因为二次项系数 " << a_y << " > 0)" << endl;
		}
		else {
			cout << "类型: 极大值 (因为二次项系数 " << a_y << " < 0)" << endl;
		}

		// 检查极值点是否在数据范围内
		if (extreme_i_y >= 0 && extreme_i_y <= indices.size() - 1) {
			cout << "极值点在数据范围内 [0, " << indices.size() - 1 << "]" << endl;
		}
		else {
			cout << "极值点不在数据范围内" << endl;
		}
	}
	else {
		cout << "Y坐标拟合为线性函数，无极值点" << endl;
	}

	// 计算拟合误差
	double x_error = 0, y_error = 0;
	for (size_t i = 0; i < indices.size(); ++i) {
		double x_fitted = a_x * i * i + b_x_val * i + c_x;
		double y_fitted = a_y * i * i + b_y_val * i + c_y;
		x_error += pow(x_fitted - x_data[i], 2);
		y_error += pow(y_fitted - y_data[i], 2);
	}

	x_error = sqrt(x_error / indices.size());
	y_error = sqrt(y_error / indices.size());

	cout << "\n=== 拟合误差 ===" << endl;
	cout << "X坐标拟合均方根误差: " << x_error << endl;
	cout << "Y坐标拟合均方根误差: " << y_error << endl;

	// 输出前几个点的拟合对比
	cout << "\n=== 前5个点的拟合对比 ===" << endl;
	for (int i = 0; i < min(5, (int)points.size()); ++i) {
		double x_fit = a_x * i * i + b_x_val * i + c_x;
		double y_fit = a_y * i * i + b_y_val * i + c_y;
		cout << "索引 " << i << ": 实际(" << x_data[i] << ", " << y_data[i]
			<< ") 拟合(" << x_fit << ", " << y_fit << ")" << endl;
	}

	vector<coeffs> coeff;
	coeffs coeff1;
	coeff1.a = a_x;
	coeff1.b = b_x_val;
	coeff1.c = c_x;
	coeff1.index = extreme_i_x;
	coeff1.value = extreme_value_x;
	coeff.push_back(coeff1);
	coeff1.a = a_y;
	coeff1.b = b_y_val;
	coeff1.c = c_y;
	coeff1.index = extreme_i_y;
	coeff1.value = extreme_value_y;
	coeff.push_back(coeff1);
	return coeff;
}

// 专门为小样本优化的IQR异常值检测
std::vector<double> removeOutliersIQRSmallSample(const std::vector<double>& data,
	double multiplier) {
	if (data.size() < 5) {
		std::cout << "数据点太少(" << data.size() << ")，不建议进行异常值剔除" << std::endl;
		return data;
	}

	std::vector<double> sorted_data = data;
	std::sort(sorted_data.begin(), sorted_data.end());

	// 对于小样本，使用更精确的分位数计算
	int n = sorted_data.size();

	// 计算Q1（第一四分位数）
	double q1_index = 0.25 * (n + 1) - 1;
	int q1_low = std::floor(q1_index);
	int q1_high = std::ceil(q1_index);
	double q1 = (q1_low == q1_high) ?
		sorted_data[q1_low] :
		sorted_data[q1_low] * (q1_high - q1_index) +
		sorted_data[q1_high] * (q1_index - q1_low);

	// 计算Q3（第三四分位数）
	double q3_index = 0.75 * (n + 1) - 1;
	int q3_low = std::floor(q3_index);
	int q3_high = std::ceil(q3_index);
	double q3 = (q3_low == q3_high) ?
		sorted_data[q3_low] :
		sorted_data[q3_low] * (q3_high - q3_index) +
		sorted_data[q3_high] * (q3_index - q3_low);

	double iqr = q3 - q1;

	// 对于小样本，使用更保守的阈值
	double adjusted_multiplier = (n <= 10) ? 2.0 : multiplier; // 小样本用2.0
	double lower_bound = q1 - adjusted_multiplier * iqr;
	double upper_bound = q3 + adjusted_multiplier * iqr;

	std::vector<double> filtered;
	int outlier_count = 0;

	for (double val : data) {
		if (val >= lower_bound && val <= upper_bound) {
			filtered.push_back(val);
		}
		else {
			std::cout << "检测到异常值: " << val
				<< " (边界: [" << lower_bound << ", " << upper_bound << "])" << std::endl;
			outlier_count++;
		}
	}

	// 如果剔除太多点，回退到更宽松的阈值
	if (filtered.size() < data.size() * 0.7 && outlier_count > 0) {
		std::cout << "剔除点过多，使用更宽松的阈值重新检测..." << std::endl;
		return removeOutliersIQRSmallSample(data, multiplier + 0.5);
	}

	std::cout << "原始数据: " << data.size() << " 个点，剔除 " << outlier_count
		<< " 个异常值，剩余 " << filtered.size() << " 个点" << std::endl;

	return filtered;
}

// 拟合一次函数 y = ax + b，返回系数 pair<a, b>
std::pair<double, double> LinearFit(const std::vector<double>& xData, const std::vector<double>& yData, char which)
{
	// 检查数据有效性
	if (xData.size() != yData.size() || xData.size() < 2)
	{
		// 返回默认值或抛出异常
		return std::make_pair(0.0, 0.0);
	}

	int n = xData.size();
	double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;

	// 计算各项和
	for (int i = 0; i < n; i++)
	{
		sumX += xData[i];
		sumY += yData[i];
		sumXY += xData[i] * yData[i];
		sumX2 += xData[i] * xData[i];
	}

	// 计算斜率a和截距b
	double denominator = n * sumX2 - sumX * sumX;
	if (fabs(denominator) < 1e-15) // 防止除零
	{
		return std::make_pair(0.0, 0.0);
	}

	double a = (n * sumXY - sumX * sumY) / denominator;
	double b = (sumY * sumX2 - sumX * sumXY) / denominator;

	if (which == 'x')
		plotLinearFit(xData, yData, make_pair(a, b), "Hori Fit");
	if (which == 'y')
		plotLinearFit(xData, yData, make_pair(a, b), "Verti Fit");

	return std::make_pair(a, b);
}

// 线性拟合绘图函数
void plotLinearFit(const std::vector<double>& xData, const std::vector<double>& yData,
	const std::pair<double, double>& coefficients,
	const std::string& title,
	int W, int H)
{
	if (xData.empty() || yData.empty() || xData.size() != yData.size()) {
		cout << "错误: 数据为空或长度不一致!" << endl;
		return;
	}

	// 将double向量转换为Point2d向量，复用现有的plotSequence基础设施
	std::vector<cv::Point2d> points;
	for (size_t i = 0; i < xData.size(); ++i) {
		points.push_back(cv::Point2d(xData[i], yData[i]));
	}

	// 计算数据范围（复用plotSequence的逻辑）
	auto mmX = std::minmax_element(xData.begin(), xData.end());
	auto mmY = std::minmax_element(yData.begin(), yData.end());
	double dx = *mmX.second - *mmX.first;
	double dy = *mmY.second - *mmY.first;
	double xMin = *mmX.first - 0.05 * dx;
	double xMax = *mmX.second + 0.05 * dx;
	double yMin = *mmY.first - 0.05 * dy;
	double yMax = *mmY.second + 0.05 * dy;

	// 创建画布（复用plotSequence的设置）
	cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

	// 绘制坐标轴（复用plotSequence的坐标轴绘制代码）
	cv::line(img, cv::Point(40, H - 40), cv::Point(W - 20, H - 40), cv::Scalar(0, 0, 0), 2);
	cv::line(img, cv::Point(40, 20), cv::Point(40, H - 40), cv::Scalar(0, 0, 0), 2);

	// 绘制刻度（复用plotSequence的刻度绘制代码）
	const int numTicks = 5;
	const int tickLength = 5;

	// X轴刻度
	for (int i = 0; i <= numTicks; ++i) {
		double xValue = xMin + i * (xMax - xMin) / numTicks;
		cv::Point2d tickPos = mapToImg(xValue, yMin, xMin, xMax, yMin, yMax, W, H);
		cv::line(img, cv::Point(tickPos.x, H - 40), cv::Point(tickPos.x, H - 40 + tickLength),
			cv::Scalar(0, 0, 0), 1);
		cv::putText(img, cv::format("%.2f", xValue), cv::Point(tickPos.x - 15, H - 20),
			cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
	}

	// Y轴刻度
	for (int i = 0; i <= numTicks; ++i) {
		double yValue = yMin + i * (yMax - yMin) / numTicks;
		cv::Point2d tickPos = mapToImg(xMin, yValue, xMin, xMax, yMin, yMax, W, H);
		cv::line(img, cv::Point(40, tickPos.y), cv::Point(40 - tickLength, tickPos.y),
			cv::Scalar(0, 0, 0), 1);
		cv::putText(img, cv::format("%.2f", yValue), cv::Point(10, tickPos.y + 5),
			cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
	}

	// 绘制数据点（复用plotSequence的散点绘制）
	for (const auto& point : points) {
		cv::Point2d p = mapToImg(point.x, point.y, xMin, xMax, yMin, yMax, W, H);
		cv::circle(img, p, 4, cv::Scalar(0, 0, 255), -1); // 红色点
	}

	// 绘制拟合直线
	double a = coefficients.first;
	double b = coefficients.second;

	// 计算直线在显示范围内的两个端点
	double y1 = a * xMin + b;
	double y2 = a * xMax + b;

	cv::Point2d p1 = mapToImg(xMin, y1, xMin, xMax, yMin, yMax, W, H);
	cv::Point2d p2 = mapToImg(xMax, y2, xMin, xMax, yMin, yMax, W, H);

	cv::line(img, p1, p2, cv::Scalar(255, 0, 0), 2); // 蓝色线

													 // 计算R?值（复用plotSequence的文本绘制）
	double meanY = std::accumulate(yData.begin(), yData.end(), 0.0) / yData.size();
	double ssTot = 0.0, ssRes = 0.0;

	for (size_t i = 0; i < xData.size(); ++i) {
		double yPred = a * xData[i] + b;
		ssTot += (yData[i] - meanY) * (yData[i] - meanY);
		ssRes += (yData[i] - yPred) * (yData[i] - yPred);
	}

	double rSquared = (ssTot > 1e-10) ? (1.0 - (ssRes / ssTot)) : 1.0;

	// 添加标题和方程（复用plotSequence的文本绘制风格）
	cv::putText(img, title, cv::Point(50, 30),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

	std::string equation = cv::format("y = %.4fx + %.4f", a, b);
	cv::putText(img, equation, cv::Point(50, 50),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);

	std::string r2Text = cv::format("R? = %.4f", rSquared);
	cv::putText(img, r2Text, cv::Point(50, 70),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 100, 0), 1);

	// 轴标签
	cv::putText(img, "X", cv::Point(W - 60, H - 10),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	cv::putText(img, "Y", cv::Point(10, 20),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

	// 显示图像
	//cv::imshow(title, img);
}
