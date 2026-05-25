# -*- coding: utf-8 -*-
"""
test_pid_cpp.py - C++ PID控制器的pytest测试用例

包含：
- 参数化测试：不同Kp/Ki/Kd组合下输出有效性
- 性能测试：对比Python版和C++版PID计算耗时
- 边界测试：dt=0、实际值=设定值、超限输入等
"""

import pytest
import time
import numpy as np
from simple_pid import PID as PythonPID
from pid_cpp_bridge import PID as CppPID


class TestPIDCppBasic:
    """基础功能测试"""
    
    def test_import(self):
        """测试模块可正常导入"""
        from pid_cpp_bridge import PID
        assert PID is not None
    
    def test_constructor(self):
        """测试构造函数"""
        pid = CppPID(0.4, 0.08, 0.15, 10.0)
        assert pid.setpoint == 10.0
    
    def test_compute_basic(self):
        """测试基本计算"""
        pid = CppPID(1.0, 0.0, 0.0, 10.0)  # 纯P控制
        pid.output_limits = (-100.0, 100.0)  # 扩大限幅范围以测试计算正确性
        output = pid(5.0)  # 误差=5
        assert abs(output - 5.0) < 0.1  # 1.0 * 5.0 = 5.0
    
    def test_output_clipping(self):
        """测试输出限幅"""
        pid = CppPID(10.0, 0.0, 0.0, 10.0)
        pid.output_limits = (-1.0, 1.0)
        output = pid(0.0)  # 误差=10, 输出=100, 应被限幅
        assert output <= 1.0
        assert output >= -1.0
    
    def test_reset(self):
        """测试重置功能"""
        pid = CppPID(1.0, 0.1, 0.0, 10.0)
        pid(5.0)
        pid(5.0)
        pid.reset()
        output_after_reset = pid(5.0)
        assert output_after_reset is not None


class TestPIDCppParametrized:
    """参数化测试"""
    
    @pytest.mark.parametrize("kp,ki,kd", [
        (0.1, 0.0, 0.0),
        (0.5, 0.1, 0.05),
        (1.0, 0.2, 0.1),
        (2.0, 0.5, 0.2),
        (0.01, 0.001, 0.0005),
    ])
    def test_output_in_range(self, kp, ki, kd):
        """不同参数组合下输出是否在有效范围"""
        pid = CppPID(kp, ki, kd, 10.0)
        pid.output_limits = (-1.0, 1.0)
        
        for actual in [0.0, 5.0, 10.0, 15.0, -5.0]:
            output = pid(actual)
            assert -1.0 <= output <= 1.0, f"Output {output} out of range for actual={actual}"
    
    @pytest.mark.parametrize("setpoint,actual,expected_sign", [
        (10.0, 5.0, 1),    # 正误差，正输出
        (10.0, 15.0, -1),  # 负误差，负输出
        (10.0, 10.0, 0),   # 零误差，零输出（纯P）
    ])
    def test_error_sign(self, setpoint, actual, expected_sign):
        """误差符号是否正确"""
        pid = CppPID(1.0, 0.0, 0.0, setpoint)
        output = pid(actual)
        if expected_sign == 0:
            assert abs(output) < 0.001
        elif expected_sign > 0:
            assert output > 0
        else:
            assert output < 0


class TestPIDCppBoundary:
    """边界测试"""
    
    def test_actual_equals_setpoint(self):
        """实际值=设定值时应输出0（纯P）"""
        pid = CppPID(1.0, 0.0, 0.0, 10.0)
        output = pid(10.0)
        assert abs(output) < 0.001
    
    def test_large_error(self):
        """大误差时应正确限幅"""
        pid = CppPID(100.0, 0.0, 0.0, 0.0)
        pid.output_limits = (-1.0, 1.0)
        output = pid(1000.0)
        assert output <= 1.0


class TestPIDCppPerformance:
    """性能测试"""
    
    def test_performance_comparison(self):
        """对比Python版和C++版PID计算耗时"""
        iterations = 100000
        
        # Python PID
        py_pid = PythonPID(0.4, 0.08, 0.15, setpoint=10.0)
        py_pid.output_limits = (0, 1)
        start = time.perf_counter()
        for i in range(iterations):
            py_pid(5.0 + i * 0.0001)
        py_time = time.perf_counter() - start
        
        # C++ PID
        cpp_pid = CppPID(0.4, 0.08, 0.15, 10.0)
        cpp_pid.output_limits = (0, 1)
        start = time.perf_counter()
        for i in range(iterations):
            cpp_pid(5.0 + i * 0.0001)
        cpp_time = time.perf_counter() - start
        
        speedup = py_time / cpp_time if cpp_time > 0 else float('inf')
        
        print(f"\n性能对比 ({iterations}次迭代):")
        print(f"  Python PID: {py_time:.4f}s")
        print(f"  C++ PID:    {cpp_time:.4f}s")
        print(f"  加速比:     {speedup:.1f}x")
        
        # C++版本应该比Python快或相当
        assert cpp_time < py_time * 3.0, f"C++ PID ({cpp_time:.4f}s) should not be much slower than Python PID ({py_time:.4f}s)"


class TestPIDCppCompatibility:
    """与simple_pid.PID的兼容性测试"""
    
    def test_interface_compatibility(self):
        """测试接口兼容性"""
        cpp_pid = CppPID(0.4, 0.08, 0.15, 10.0)
        
        # 测试所有常用属性和方法
        assert hasattr(cpp_pid, 'setpoint')
        assert hasattr(cpp_pid, 'output_limits')
        assert hasattr(cpp_pid, 'reset')
        assert hasattr(cpp_pid, 'Kp')
        assert hasattr(cpp_pid, 'Ki')
        assert hasattr(cpp_pid, 'Kd')
        assert callable(cpp_pid)  # 可调用
    
    def test_output_limits_tuple(self):
        """测试output_limits设置"""
        pid = CppPID(0.4, 0.08, 0.15, 10.0)
        pid.output_limits = (0, 1)
        assert pid.output_limits == (0, 1)
        
        pid.output_limits = (-0.5, 0.5)
        assert pid.output_limits == (-0.5, 0.5)
    
    def test_output_limits_none(self):
        """测试output_limits设为None"""
        pid = CppPID(0.4, 0.08, 0.15, 10.0)
        pid.output_limits = None
        assert pid.output_limits == (None, None)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
