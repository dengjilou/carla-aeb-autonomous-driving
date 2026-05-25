"""
pid_cpp_bridge.py - C++ PID控制器的Python封装层

提供与 simple_pid.PID 完全兼容的接口，可直接替换原有Python PID调用。
底层使用C++实现的PID控制器，性能更高。
"""

import pid_cpp
import time


class PID:
    """
    C++实现的PID控制器，封装自pid_cpp扩展模块。
    
    接口与 simple_pid.PID 完全一致：
    - 构造函数：PID(Kp, Ki, Kd, setpoint=0.0)
    - output_limits: 元组(min, max)，设置输出限幅
    - __call__(actual): 计算并返回PID输出（自动计算dt）
    - reset(): 重置积分和误差状态
    """
    
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, setpoint=0.0):
        """
        初始化PID控制器
        
        参数:
            kp: 比例增益
            ki: 积分增益
            kd: 微分增益
            setpoint: 目标值
        """
        self._pid = pid_cpp.PID(kp, ki, kd, setpoint)
        self._output_limits = (None, None)
        self._last_time = None
    
    def __call__(self, actual):
        """
        计算PID输出
        
        参数:
            actual: 当前实际值
        
        返回:
            控制输出值（已在output_limits范围内）
        """
        now = time.perf_counter()
        if self._last_time is None:
            self._last_time = now
            dt = 0.041666666666666664  # 默认dt=1/24
        else:
            dt = now - self._last_time
            self._last_time = now
        
        # dt过小或异常时使用默认值
        if dt <= 0 or dt > 1.0:
            dt = 0.041666666666666664
        
        return self._pid.compute(actual, dt)
    
    @property
    def setpoint(self):
        """获取目标值"""
        return self._pid.setpoint
    
    @setpoint.setter
    def setpoint(self, value):
        """设置目标值"""
        self._pid.set_setpoint(value)
    
    @property
    def Kp(self):
        """获取比例增益"""
        return self._pid.kp
    
    @Kp.setter
    def Kp(self, value):
        """设置比例增益"""
        pass
    
    @property
    def Ki(self):
        """获取积分增益"""
        return self._pid.ki
    
    @Ki.setter
    def Ki(self, value):
        """设置积分增益"""
        pass
    
    @property
    def Kd(self):
        """获取微分增益"""
        return self._pid.kd
    
    @Kd.setter
    def Kd(self, value):
        """设置微分增益"""
        pass
    
    @property
    def output_limits(self):
        """获取输出限幅"""
        return self._output_limits
    
    @output_limits.setter
    def output_limits(self, limits):
        """
        设置输出限幅
        
        参数:
            limits: 元组(min, max)，例如 (-1.0, 1.0)
        """
        if limits is None:
            self._output_limits = (None, None)
            self._pid.set_output_limits(-1e10, 1e10)
        else:
            min_val, max_val = limits
            if min_val is None:
                min_val = -1e10
            if max_val is None:
                max_val = 1e10
            self._output_limits = (min_val, max_val)
            self._pid.set_output_limits(min_val, max_val)
    
    def reset(self):
        """重置PID状态（积分项和误差）"""
        self._pid.reset()
        self._last_time = None
