#include "img_roi.h"

bool img_roi::createRoiMasks(cv::Mat& inputImg, cv::Mat& gobal_skeleton,
                      int y_upper_start, int y_upper_end,
                      int y_lower_start, int y_lower_end,
                      int x_upper_start, int x_upper_end,
                      int x_lower_start, int x_lower_end,
                      cv::Mat& upper_mask, cv::Mat& lower_mask,
                      cv::Mat& upper_roi, cv::Mat& lower_roi)
{
    // 边界有效性检查
    if (y_upper_start > y_upper_end || y_lower_start > y_lower_end ||
        x_upper_start >= x_upper_end || x_lower_start >= x_lower_end) {
        std::cout << "边界范围无效" << std::endl;
        return false;
    }

    y_upper_start = std::max(0, min(y_upper_start, inputImg.rows - 1));
    y_upper_end = std::max(0, min(inputImg.rows - 1, y_upper_end));
    if(y_upper_start >= y_upper_end){
        cerr << "上半部分 y 范围无效: start >= end (" << y_upper_start << " >= " << y_upper_end << ")" << endl;
        return false;
    }
    y_lower_start = std::max(0, min(y_lower_start, inputImg.rows - 1));
    y_lower_end = std::max(0, min(inputImg.rows - 1, y_lower_end));
    if(y_lower_start >= y_lower_end){
        cerr << "下半部分 y 范围无效: start >= end (" << y_lower_start << " >= " << y_lower_end << ")" << endl;
        return false;
    }
    x_upper_start = std::max(0, min(x_upper_start, inputImg.cols - 1));
    x_upper_end = std::max(0, min(inputImg.cols - 1, x_upper_end));
    if(x_upper_start >= x_upper_end){
        cerr << "上半部分 x 范围无效: start >= end (" << x_upper_start << " >= " << x_upper_end << ")" << endl;
        return false;
    }
    x_lower_start = std::max(0, min(x_lower_start, inputImg.cols - 1));
    x_lower_end = std::max(0, min(inputImg.cols - 1, x_lower_end));
    if(x_lower_start >= x_lower_end){
        cerr << "下半部分 x 范围无效: start >= end (" << x_lower_start << " >= " << x_lower_end << ")" << endl;
        return false;
    }
    //输出调整后的边界值
    cout << "调整后的边界值: \n"
        << "上部分区域: y范围 [" << y_upper_start << ", " << y_upper_end << "]\n"
        << "下部分区域: y范围 [" << y_lower_start << ", " << y_lower_end << "]\n"
        << "上部分区域: x范围: [" << x_upper_start << ", " << x_upper_end << "]\n"
        << "下部分区域: x范围: [" << x_lower_start << ", " << x_lower_end << "]\n" << endl;


    // 初始化掩码矩阵
    upper_mask = cv::Mat::zeros(inputImg.size(), CV_8UC1);
    lower_mask = cv::Mat::zeros(inputImg.size(), CV_8UC1);

    //// 像素级掩码生成
    //for (int y = 0; y < inputImg.rows; ++y) {
    //    for (int x = 0; x < inputImg.cols; ++x) {
    //        // 上半区域掩码
    //        if (x >= x_upper_start && x <= x_upper_end &&
    //            y >= y_upper_start && y <= y_upper_end) {
    //            upper_mask.at<uchar>(y, x) = 255;
    //        }
    //        // 下半区域掩码
    //        if (x >= x_lower_start && x <= x_lower_end &&
    //            y >= y_lower_start && y <= y_lower_end) {
    //            lower_mask.at<uchar>(y, x) = 255;
    //        }
    //    }
    //}

    // 使用矩形填充，比逐像素快得多
    cv::rectangle(upper_mask,
        cv::Point(x_upper_start, y_upper_start),
        cv::Point(x_upper_end, y_upper_end),
        cv::Scalar(255), cv::FILLED);

    cv::rectangle(lower_mask,
        cv::Point(x_lower_start, y_lower_start),
        cv::Point(x_lower_end, y_lower_end),
        cv::Scalar(255), cv::FILLED);

    // 应用掩码到骨架图,low_mask复制到roi
    gobal_skeleton.copyTo(upper_roi, upper_mask);
    gobal_skeleton.copyTo(lower_roi, lower_mask);
    //cv::imshow("Upper Region", upper_roi);
    //cv::imshow("Lower Region", lower_roi);

    return true;
}