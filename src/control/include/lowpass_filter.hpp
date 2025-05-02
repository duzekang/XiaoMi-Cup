#pragma once
#include <cmath>

// 自定义低通滤波器实现
class LowPassFilter {
private:
    double alpha;  // 滤波系数
    double y_prev; // 前一次的输出值
    bool first_sample = true;

public:
    // 构造函数：截止频率和采样时间
    LowPassFilter(double cutoff_freq, double dt) {
        set_cutoff(cutoff_freq, dt);
    }

    void set_cutoff(double cutoff_freq, double dt) {
        double rc = 1.0 / (2 * M_PI * cutoff_freq);
        alpha = dt / (rc + dt);
    }

    double filter(double x) {
        if (first_sample) {
            first_sample = false;
            y_prev = x;
            return x;
        }
        double y = alpha * x + (1 - alpha) * y_prev;
        y_prev = y;
        return y;
    }

    void reset() {
        first_sample = true;
    }
};