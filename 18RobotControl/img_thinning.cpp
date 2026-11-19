#include "img_thinning.h"
#include "img_debug.h"

img_thinning::img_thinning(){}

img_thinning::~img_thinning(){}

void img_thinning::zhangSuenThinning(cv::Mat& src, cv::Mat& dst) {
    // 1. 预分配内存避免重复创建
    static cv::Mat prev, marker, diff;
    if (prev.empty() || prev.size() != src.size()) {
        prev = cv::Mat::zeros(src.size(), CV_8UC1);
        marker = cv::Mat::zeros(src.size(), CV_8UC1);
    }
    
    // 2. 直接使用输入图像避免克隆开销
    cv::Mat thinned;
    src.convertTo(thinned, CV_8UC1, 1.0/255); // 归一化操作
    
    const int MAX_ITERS = 100;
    int iter_count = 0;
    bool changed;

    // 3. 优化像素访问：使用行指针缓存
    auto applyIteration = [&](int iter_type) {
        marker.setTo(0);
        uchar* marker_data = marker.ptr<uchar>();
        int step = marker.step1();

        for (int i = 1; i < thinned.rows - 1; i++) {
            const uchar* prev_row = thinned.ptr<uchar>(i-1);
            const uchar* curr_row = thinned.ptr<uchar>(i);
            const uchar* next_row = thinned.ptr<uchar>(i+1);
            
            for (int j = 1; j < thinned.cols - 1; j++) {
                if (curr_row[j] == 0) continue; // 跳过背景像素
                
                // 4. 邻域像素访问优化（减少指针运算）
                uchar p2 = prev_row[j];
                uchar p3 = prev_row[j+1];
                uchar p4 = curr_row[j+1];
                uchar p5 = next_row[j+1];
                uchar p6 = next_row[j];
                uchar p7 = next_row[j-1];
                uchar p8 = curr_row[j-1];
                uchar p9 = prev_row[j-1];

                // 5. 优化条件判断（无分支计算）
                int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                        (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                        (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                        (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
                
                int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                
                // 6. 使用位运算替代乘法
                int m1 = (iter_type == 1) ? 
                         (p2 & p4 & p6) : 
                         (p2 & p4 & p8);
                
                int m2 = (iter_type == 1) ? 
                         (p4 & p6 & p8) : 
                         (p2 & p6 & p8);

                if (A == 1 && (B >= 2 && B <= 6) && !m1 && !m2) {
                    marker_data[i*step + j] = 1;
                }
            }
        }
        thinned &= ~marker; // 原地操作
    };

    // 7. 优化迭代终止条件
    do {
        changed = false;
        applyIteration(1);
        applyIteration(2);
        
        cv::absdiff(thinned, prev, diff);
        if (cv::countNonZero(diff) > 0) {
            changed = true;
            thinned.copyTo(prev);
        }
        iter_count++;
    } while (changed && iter_count < MAX_ITERS);

    // 8. 直接输出到dst，避免额外克隆
    thinned.convertTo(dst, CV_8UC1, 255);
    if (g_showImageDebug) cv::imshow("thinned", dst);
}
