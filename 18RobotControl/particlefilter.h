#pragma once
#include<opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>

using namespace cv;
using namespace std;

// 粒子结构体
struct Particle {
    double k;       // 直线斜率k
    double b;       // 直线截距b
    double weight;  // 粒子权重
};

class particlefilter {
private:
    // 粒子滤波器参数
    const int NUM_PARTICLES = 200;      // 粒子数量
    const double DELTA_K = 0.15;        // 粒子参数斜率k变化范围       0.15
    const double DELTA_B = 120.0;       // 粒子参数截距b变化范围       120.0
    const double SIGMA1 = 0.2;          // 斜率噪声标准差              1.0
    const double SIGMA2 = 3.0;          // 截距噪声标准差      如果粒子在转移过程中没有引入噪声，那么所有粒子都会根据相同的状态转移方程移动，导致粒子群过于集中（粒子退化）
    const double WEIGHT_SIGMA = 3.0;    // 粒子权重标准差
    const int TEMPLATE_WIDTH = 6;      // 带状模板宽度(像素)            10

    // 状态变量
    cv::Mat prevImage;                  // 前一帧图像


    // 随机数生成器
    std::default_random_engine generator;

public:
    std::vector<Particle> particles;
    double current_k = 0.0;
    double current_b = 0.0;
    double prev_k = 0.0;
    double prev_b = 0.0;
    double prev_prev_k = 0.0;
    double prev_prev_b = 0.0;

    // 初始化粒子滤波器
    void pfinitialize(cv::Mat& roiImage, double k, double b);
    // 处理新帧
    void pfprocessFrame(cv::Mat& roiImage);

private:
    // 计算粒子权重
    double computeParticleWeight(cv::Mat& image, Particle& p);
    // 重采样粒子
    void resampleParticles();
    // 估计当前状态
    void estimateState();
    // 粒子滤波
    //void particleFilter(const cv::Mat& image);
};
