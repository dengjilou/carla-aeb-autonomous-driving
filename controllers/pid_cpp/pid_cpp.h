#ifndef PID_CPP_H
#define PID_CPP_H

class PID {
public:
    // 构造函数：Kp-比例增益, Ki-积分增益, Kd-微分增益, setpoint-目标值
    PID(double kp, double ki, double kd, double setpoint);
    
    // 计算控制输出：actual-实际值, dt-时间步长
    double compute(double actual, double dt);
    
    // 设置输出限幅
    void set_output_limits(double min_val, double max_val);
    
    // 重置积分项
    void reset();
    
    // 设置新的目标值
    void set_setpoint(double new_setpoint);
    
    // 获取当前误差
    double get_error() const;
    
    // 获取PID参数
    double get_kp() const { return kp_; }
    double get_ki() const { return ki_; }
    double get_kd() const { return kd_; }
    double get_setpoint() const { return setpoint_; }

private:
    double kp_;          // 比例增益
    double ki_;          // 积分增益
    double kd_;          // 微分增益
    double setpoint_;    // 目标值
    
    double prev_error_;      // 上一时刻误差
    double integral_;        // 积分累积
    double min_output_;      // 输出下限
    double max_output_;      // 输出上限
};

#endif // PID_CPP_H
