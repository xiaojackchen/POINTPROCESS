#pragma once

#include <iostream>
#include <cmath>
#include <random>
#include <iomanip>
#include <opencv2/opencv.hpp>

using namespace cv;

class KalmanFilter1D {
private:
    // 状态向量: [位置, 速度]
    double position;
    double velocity;
    
    // 状态协方差矩阵 (2x2)
    double P11; // 位置方差
    double P12; // 位置-速度协方差
    double P22; // 速度方差
    
    // 状态转移矩阵 (2x2)
    double F11, F12;
    double F21, F22;
    
    // 过程噪声协方差 (2x2)
    double Q11, Q12;
    double Q21, Q22;
    
    // 观测矩阵 (1x2)
    double H1, H2;
    
    // 观测噪声方差 (标量)
    double R;
    
    // 卡尔曼增益 (2x1)
    double K1, K2;

    // 时间步长
    double dt;

    // 二维点专用：x/y 滤波器指针与标志
    std::unique_ptr<KalmanFilter1D> kf_x_;
    std::unique_ptr<KalmanFilter1D> kf_y_;
    bool point_filter_init_ = false;

public:
    KalmanFilter1D(double init_pos, double init_vel, double dt, double process_var, double meas_var);

    // 二维点接口（首次调用自动初始化内部两个1D滤波器）
    void KalmanFilter1D_2(cv::Point2d& point);

    void predict();

    void update(double measurement);

    double getPosition() const {
        return position;
    }

    double getVelocity() const {
        return velocity;
    }

    double getPositionVariance() const {
        return P11;
    }

    void setProcessNoise(double variance);

    // 稳定性检查
    bool isStable() const {
        return std::isfinite(position) && std::isfinite(velocity) &&
            std::isfinite(P11) && std::isfinite(P22) &&
            P11 > 0 && P22 > 0 &&
            P11 < 1e6 && P22 < 1e6; // 防止协方差爆炸
    }

    // 重置二维点滤波器
    void resetPointFilter() {
        point_filter_init_ = false;
        kf_x_.reset();
        kf_y_.reset();
        std::cout << "Point filter reset" << std::endl;
    }

    // 重置单个1D滤波器
    void reset(double new_position, double new_velocity = 0.0) {
        position = new_position;
        velocity = new_velocity;
        P11 = 10.0;  // 重置为较大的初始不确定性
        P12 = 0.0;
        P22 = 10.0;
        std::cout << "1D Filter reset to position: " << new_position << std::endl;
    }

    // 安全更新方法
    void safeUpdate(double measurement) {
        if (!isStable()) {
            std::cout << "WARNING: Kalman filter unstable, resetting..." << std::endl;
            reset(measurement); // 重置到当前测量值
        }
        update(measurement);
    }
};
