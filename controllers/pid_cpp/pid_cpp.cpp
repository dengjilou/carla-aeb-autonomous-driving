#include "pid_cpp.h"
#include <algorithm>

// 构造函数初始化列表
PID::PID(double kp, double ki, double kd, double setpoint)
    : kp_(kp), ki_(ki), kd_(kd), setpoint_(setpoint),
      prev_error_(0.0), integral_(0.0),
      min_output_(-1.0), max_output_(1.0) {}

// PID计算公式：output = Kp*e + Ki*∫e*dt + Kd*de/dt
double PID::compute(double actual, double dt) {
    // 防御性编程：dt为0或负数时返回0，避免除零错误
    if (dt <= 0.0) {
        return 0.0;
    }
    
    // 计算当前误差
    double error = setpoint_ - actual;
    
    // 积分项累积（限幅防止积分饱和）
    integral_ += error * dt;
    
    // 微分项：误差变化率
    double derivative = (error - prev_error_) / dt;
    
    // PID输出 = 比例项 + 积分项 + 微分项
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    
    // 输出限幅
    output = std::max(min_output_, std::min(max_output_, output));
    
    // 更新上一时刻误差
    prev_error_ = error;
    
    return output;
}

// 设置输出限幅范围
void PID::set_output_limits(double min_val, double max_val) {
    min_output_ = min_val;
    max_output_ = max_val;
}

// 重置积分项和上一时刻误差
void PID::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}

// 更新目标值
void PID::set_setpoint(double new_setpoint) {
    setpoint_ = new_setpoint;
}

// 获取当前误差
double PID::get_error() const {
    return setpoint_ - (setpoint_ - prev_error_); // 返回最后一次计算的误差
}
