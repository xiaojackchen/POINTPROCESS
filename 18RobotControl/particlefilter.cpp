#include "particlefilter.h"

// 初始化粒子滤波器
void particlefilter::pfinitialize(cv::Mat& roiImage, double k, double b) {
    double k0 = k;
    double b0 = b;
    
    // 步骤2: 粒子初始化
    particles.clear();
    std::uniform_real_distribution<double> k_dist(k0 - DELTA_K, k0 + DELTA_K);      //指定范围内均匀分布的随机实数
    std::uniform_real_distribution<double> b_dist(b0 - DELTA_B, b0 + DELTA_B);
    
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        // 利用default_random_engine generator生成范围在(k0 - DELTA_K, k0 + DELTA_K)和(b0 - DELTA_B, b0 + DELTA_B)间的随机整数
        particles.push_back({k_dist(generator), b_dist(generator), 1.0/NUM_PARTICLES});
    }
    
    // 设置初始状态
    current_k = k0;
    current_b = b0;
    prev_k = k0;
    prev_b = b0;
    prev_prev_k = k0;
    prev_prev_b = b0;
    prevImage = roiImage.clone();

    cout << "Initializing particles..." << endl;
    cout << "line" << "y = " << current_k << "x + " << current_b << endl;
}

// 处理新帧
void particlefilter::pfprocessFrame(cv::Mat& roiImage) {
    // 步骤3: 粒子转移
    std::normal_distribution<double> k_noise(0, SIGMA1);
    std::normal_distribution<double> b_noise(0, SIGMA2);
    
    for (auto& p : particles) {
        // 二阶自回归模型: X_t = 2*X_{t-1} - X_{t-2} + V_{t-1}
        double new_k = 2*prev_k - prev_prev_k + k_noise(generator);
        double new_b = 2*prev_b - prev_prev_b + b_noise(generator);
        
        p.k =  new_k;
        p.b =  new_b;
    }
    
    // 步骤4: 粒子权重计算
    double totalWeight = 0.0;
    for (auto& p : particles) {
        p.weight = computeParticleWeight(roiImage, p);
        totalWeight += p.weight;
    }
    
    // 归一化权重
    if (totalWeight > 0) {
        for (auto& p : particles) {
            p.weight /= totalWeight;
        }
    }
    
    // 步骤5: 重采样 (系统重采样)
    resampleParticles();
    
    // 步骤6: 状态估计
    estimateState();
    
    // 更新状态
    prev_prev_k = prev_k;
    prev_prev_b = prev_b;
    prev_k = current_k;
    prev_b = current_b;
    prevImage = roiImage.clone();

    cout << "Particle lines: " << endl;
    cout << "line" << "y = " << current_k << "x + " << current_b << endl;
}

// 计算粒子权重
double particlefilter::computeParticleWeight(cv::Mat& image, Particle& p) {
    // 创建带状模板 (直线两侧各TEMPLATE_WIDTH/2像素)
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    const int halfWidth = TEMPLATE_WIDTH / 2;
    
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            // 计算点到直线的距离
            double distance = std::abs(p.k * x - y + p.b) / std::sqrt(p.k * p.k + 1);
            if (distance <= halfWidth) {
                //在OpenCV中，矩阵元素的访问顺序是 matrix.at<type>(row, col)，即先行后列，对应到图像坐标系中就是 at<type>(y, x)
                mask.at<uchar>(y, x) = 255;
            }
        }
    }
    
    // 计算模板区域的平均灰度值
    cv::Scalar meanVal = cv::mean(image, mask);
    //cv::imshow("line mask", mask);
    double S_t = meanVal[0];
    
    // 计算权重 (论文公式6)
    double S_ref = 255; // 参考值，根据实际图像调整
    double diff = 1.0 - (S_ref / S_t);
    return std::exp(-(diff * diff) / (2 * WEIGHT_SIGMA * WEIGHT_SIGMA));
}


// 重采样粒子
void particlefilter::resampleParticles() {
    // 1. 计算累积分布函数(CDF)
    std::vector<double> cdf(particles.size());
    cdf[0] = particles[0].weight;
    for (int i = 1; i < particles.size(); ++i) {
        cdf[i] = cdf[i - 1] + particles[i].weight;
    }
    
    // 2. 生成均匀分布随机数（修正关键点）
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::vector<double> u(NUM_PARTICLES);
    for (int m = 0; m < NUM_PARTICLES; ++m) {
        u[m] = dist(generator);
    }
    std::sort(u.begin(), u.end());  // 排序便于高效查找
    
    // 3. 系统重采样
    std::vector<Particle> newParticles;
    int i = 0;
    for (int m = 0; m < NUM_PARTICLES; ++m) {
        // 找到满足 CDF_{i-1} < u_m <= CDF_i 的索引
        while (i < cdf.size() - 1 && u[m] > cdf[i]) {
            i++;
        }
        newParticles.push_back(particles[i]);
    }
    
    // 4. 权重重置（新增重要步骤）
    particles = newParticles;
    for (auto& p : particles) {
        p.weight = 1.0 / NUM_PARTICLES;
    }
}

// 估计当前状态
void particlefilter::estimateState() {
    double sum_k = 0.0;
    double sum_b = 0.0;
    double sum_weight = 0.0;
    
    for (const auto& p : particles) {
        sum_k += p.k * p.weight;
        sum_b += p.b * p.weight;
        sum_weight += p.weight;
    }
    
    if (sum_weight > 0) {
        current_k = sum_k / sum_weight;
        current_b = sum_b / sum_weight;
    }
}

//void WeldSeamTracker::particleFilter(const cv::Mat& roiImage) {
//    // 创建跟踪器
//    WeldSeamTracker tracker;
//    
//}