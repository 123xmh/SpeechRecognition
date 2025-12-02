#include "head_tracking/kalman_filter.hpp"

KalmanFilter::KalmanFilter(float process_noise, float measurement_noise) 
    : Q(process_noise), R(measurement_noise), initialized(false) {}

void KalmanFilter::init(float initial_value) {
    x = initial_value;
    P = 1.0;  // 初始估计协方差
    initialized = true;
}

float KalmanFilter::update(float measurement) {
    if (!initialized) {
        init(measurement);
        return measurement;
    }
    
    // 预测步骤
    float x_pred = x;      // 状态预测 (无控制输入)
    float P_pred = P + Q;  // 协方差预测
    
    // 更新步骤
    float K = P_pred / (P_pred + R);  // 卡尔曼增益
    x = x_pred + K * (measurement - x_pred);  // 状态更新
    P = (1 - K) * P_pred;  // 协方差更新
    
    return x;
}

float KalmanFilter::get_value() const {
    return x;
}