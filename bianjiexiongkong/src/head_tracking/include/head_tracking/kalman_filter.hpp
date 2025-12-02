#pragma once

class KalmanFilter {
public:
    KalmanFilter(float process_noise = 0.01, float measurement_noise = 0.1);
    
    void init(float initial_value);
    float update(float measurement);
    float get_value() const;

private:
    float x;  // 状态估计
    float P;  // 估计协方差
    float Q;  // 过程噪声
    float R;  // 测量噪声
    bool initialized;  // 是否初始化标志
};