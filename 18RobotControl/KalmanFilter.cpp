#include "KalmanFilter.h"

using namespace std;
using namespace cv;

/**
 * 初始化单维卡尔曼滤波器 (仅X方向)
 *
 * @param init_pos    初始位置
 * @param init_vel    初始速度
 * @param dt          时间步长 (秒)
 * @param process_var 过程噪声方差 (运动不确定性)
 * @param meas_var    测量噪声方差 (传感器不确定性)
 */
KalmanFilter1D::KalmanFilter1D(double init_pos, double init_vel, double dt,
    double process_var, double meas_var)
    : position(init_pos), velocity(init_vel), dt(dt)
{
    // 初始协方差矩阵 (较大的不确定性)
    P11 = 10.0; // 位置方差
    P12 = 0.0;  // 位置-速度协方差
    P22 = 10.0; // 速度方差

    // 状态转移矩阵 (匀速运动模型)
    F11 = 1.0; F12 = dt;
    F21 = 0.0; F22 = 1.0;

    // 过程噪声协方差 (基于随机加速度模型)
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Q11 = dt4 / 4.0 * process_var;
    Q12 = dt3 / 2.0 * process_var;
    Q21 = Q12; // 对称矩阵
    Q22 = dt2 * process_var;

    // 观测矩阵 (只能观测位置)
    H1 = 1.0;
    H2 = 0.0;

    // kalman增益
    K1 = 0.0;
    K2 = 0.0;

    // 观测噪声方差
    R = meas_var;
}

/**
 * 预测步骤 (基于运动模型)
 */
//void KalmanFilter1D::predict() {
//    // 状态预测: x = F * x
//    double new_position = F11 * position + F12 * velocity;
//    double new_velocity = F21 * position + F22 * velocity;
//
//    // 调试输出 - 添加这一部分
//    std::cout << "  PREDICT DEBUG - Before: pos=" << position << ", vel=" << velocity
//        << ", After: pos=" << new_position << ", vel=" << new_velocity << std::endl;
//
//    position = new_position;
//    velocity = new_velocity;
//
//    // 协方差预测: P = F * P * F^T + Q
//    double new_P11 = F11 * (F11 * P11 + F12 * P12) + F12 * (F11 * P12 + F12 * P22) + Q11;
//    double new_P12 = F11 * (F11 * P12 + F12 * P22) + F12 * (F21 * P12 + F22 * P22) + Q12;
//    double new_P22 = F21 * (F21 * P11 + F22 * P12) + F22 * (F21 * P12 + F22 * P22) + Q22;
//
//    P11 = new_P11;
//    P12 = new_P12;
//    P22 = new_P22;
//}
void KalmanFilter1D::predict() {
    // 状态预测
    double new_position = F11 * position + F12 * velocity;
    double new_velocity = F21 * position + F22 * velocity;

    position = new_position;
    velocity = new_velocity;

    // 协方差预测
    double new_P11 = F11 * (F11 * P11 + F12 * P12) + F12 * (F11 * P12 + F12 * P22) + Q11;
    double new_P12 = F11 * (F11 * P12 + F12 * P22) + F12 * (F21 * P12 + F22 * P22) + Q12;
    double new_P22 = F21 * (F21 * P11 + F22 * P12) + F22 * (F21 * P12 + F22 * P22) + Q22;

    P11 = new_P11;
    P12 = new_P12;
    P22 = new_P22;

    // 确保协方差不会太小（保持滤波器灵敏度）
    if (P11 < 1e-6) P11 = 1e-6;
    if (P22 < 1e-6) P22 = 1e-6;
}

/**
 * 更新步骤 (使用传感器测量值)
 *
 * @param measurement 测量位置
 */
//void KalmanFilter1D::update(double measurement) {
//    // 计算残差: y = z - H * x
//    double residual = measurement - (H1 * position + H2 * velocity);
//
//    // 计算残差协方差: S = H * P * H^T + R
//    double S = H1 * (H1 * P11 + H2 * P12) + H2 * (H1 * P12 + H2 * P22) + R;
//
//    // 计算卡尔曼增益: K = P * H^T * S^-1
//    K1 = (P11 * H1 + P12 * H2) / S;
//    K2 = (P12 * H1 + P22 * H2) / S;
//
//    // 调试输出 - 添加这一部分
//    std::cout << "  UPDATE DEBUG - Residual: " << residual
//        << ", S: " << S << ", K1: " << K1 << ", K2: " << K2
//        << ", P11: " << P11 << ", R: " << R << std::endl;
//
//    // 更新状态估计: x = x + K * y
//    position += K1 * residual;
//    velocity += K2 * residual;
//
//    // 更新协方差估计: P = (I - K * H) * P
//    double I_minus_KH11 = 1.0 - K1 * H1;
//    double I_minus_KH12 = -K1 * H2;
//    double I_minus_KH21 = -K2 * H1;
//    double I_minus_KH22 = 1.0 - K2 * H2;
//
//    double new_P11 = I_minus_KH11 * (I_minus_KH11 * P11 + I_minus_KH12 * P12) +
//        I_minus_KH12 * (I_minus_KH11 * P12 + I_minus_KH12 * P22);
//    double new_P12 = I_minus_KH11 * (I_minus_KH21 * P11 + I_minus_KH22 * P12) +
//        I_minus_KH12 * (I_minus_KH21 * P12 + I_minus_KH22 * P22);
//    double new_P22 = I_minus_KH21 * (I_minus_KH21 * P11 + I_minus_KH22 * P12) +
//        I_minus_KH22 * (I_minus_KH21 * P12 + I_minus_KH22 * P22);
//
//    P11 = new_P11;
//    P12 = new_P12;
//    P22 = new_P22;
//}

void KalmanFilter1D::update(double measurement) {
    // 先检查稳定性
    if (!isStable()) {
        std::cout << "WARNING: Filter unstable before update, resetting..." << std::endl;
        reset(measurement);
        return;
    }

    // 计算残差: y = z - H * x
    double residual = measurement - (H1 * position + H2 * velocity);

    // 计算残差协方差: S = H * P * H^T + R
    double S = H1 * (H1 * P11 + H2 * P12) + H2 * (H1 * P12 + H2 * P22) + R;

    // 检查S是否为非正数（数值问题）
    if (S <= 0) {
        std::cout << "WARNING: Residual covariance S <= 0, resetting filter..." << std::endl;
        reset(measurement);
        return;
    }

    // 计算卡尔曼增益: K = P * H^T * S^-1
    K1 = (P11 * H1 + P12 * H2) / S;
    K2 = (P12 * H1 + P22 * H2) / S;

    // 调试输出 - 添加这一部分
    std::cout << "  UPDATE DEBUG - Residual: " << residual
        << ", S: " << S << ", K1: " << K1 << ", K2: " << K2
        << ", P11: " << P11 << ", R: " << R << std::endl;

    // 更新状态估计: x = x + K * y
    position += K1 * residual;
    velocity += K2 * residual;

    // 使用数值稳定的协方差更新公式
    // P = (I - K * H) * P
    // 更稳定的实现方式
    double temp1 = P11 - K1 * (H1 * P11 + H2 * P12);
    double temp2 = P12 - K1 * (H1 * P12 + H2 * P22);
    P22 = P22 - K2 * (H1 * P12 + H2 * P22);
    P11 = temp1;
    P12 = temp2;

    // 确保协方差矩阵保持正定性（数值稳定）
    if (P11 < 1e-6) P11 = 1e-6;
    if (P22 < 1e-6) P22 = 1e-6;

    // 检查更新后是否稳定
    if (!isStable()) {
        std::cout << "WARNING: Filter became unstable after update, resetting..." << std::endl;
        reset(measurement);
    }

    // 调试输出
    std::cout << "  UPDATE DEBUG - Residual: " << residual
        << ", S: " << S << ", K1: " << K1 << ", K2: " << K2
        << ", P11: " << P11 << ", P22: " << P22 << std::endl;
}

/**
 * 设置过程噪声参数 (用于动态调整滤波器)
 */
void KalmanFilter1D::setProcessNoise(double variance) {
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Q11 = dt4 / 4.0 * variance;
    Q12 = dt3 / 2.0 * variance;
    Q21 = Q12;
    Q22 = dt2 * variance;
}

//======================== 二维点实现（合并进来） ========================
//void KalmanFilter1D::KalmanFilter1D_2(cv::Point2d& point) {
//    cv::Point2d original_point = point; // 保存原始值
//    if (!point_filter_init_) {
//        kf_x_ = std::make_unique<KalmanFilter1D>(point.x, 0.0, dt, 0.1, 0.5); // 复用本类
//        kf_y_ = std::make_unique<KalmanFilter1D>(point.y, 0.0, dt, 0.1, 0.5);
//        point_filter_init_ = true;
//        // 第一次调用时也进行滤波
//        kf_x_->predict(); kf_x_->update(point.x); point.x = kf_x_->getPosition();
//        kf_y_->predict(); kf_y_->update(point.y); point.y = kf_y_->getPosition();
//    }
//    else {
//        kf_x_->predict(); kf_x_->update(point.x); point.x = kf_x_->getPosition();
//        kf_y_->predict(); kf_y_->update(point.y); point.y = kf_y_->getPosition();
//        // 调试输出
//        std::cout << "Original: (" << original_point.x << ", " << original_point.y << ")"
//            << " -> Filtered: (" << point.x << ", " << point.y << ")" << std::endl;
//    }
//}
void KalmanFilter1D::KalmanFilter1D_2(cv::Point2d& point) {
    cv::Point2d original_point = point;

    if (!point_filter_init_) {
        std::cout << "=== Initializing 2D Kalman Filter ===" << std::endl;

        // 使用更稳定的参数
        double process_noise = 0.01;    // 减小过程噪声
        double measurement_noise = 0.03; // 测量噪声 0.3

        kf_x_ = std::make_unique<KalmanFilter1D>(point.x, 0.0, 0.033, process_noise, measurement_noise);
        kf_y_ = std::make_unique<KalmanFilter1D>(point.y, 0.0, 0.033, process_noise, measurement_noise);
        point_filter_init_ = true;

        // 第一次也执行滤波
        kf_x_->predict(); kf_x_->update(original_point.x); point.x = kf_x_->getPosition();
        kf_y_->predict(); kf_y_->update(original_point.y); point.y = kf_y_->getPosition();
    }
    else {
        kf_x_->predict(); kf_x_->safeUpdate(original_point.x); point.x = kf_x_->getPosition();
        kf_y_->predict(); kf_y_->safeUpdate(original_point.y); point.y = kf_y_->getPosition();
    }

    std::cout << "Original: (" << original_point.x << ", " << original_point.y << ")"
        << " -> Filtered: (" << point.x << ", " << point.y << ")" << std::endl;
}
