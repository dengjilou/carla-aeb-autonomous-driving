# -*- coding: utf-8 -*-
from distutils.core import setup, Extension
import os
import sys

# 获取Python头文件路径
python_include = os.path.join(sys.prefix, 'include')

# C++扩展模块定义
pid_cpp_module = Extension(
    'pid_cpp',
    sources=['pid_cpp.cpp', 'pid_cpp_module.cpp'],
    include_dirs=[os.path.dirname(os.path.abspath(__file__))],
    extra_compile_args=['/std:c++14', '/O2'],  # MSVC优化选项
    language='c++'
)

setup(
    name='pid_cpp',
    version='1.0',
    description='C++ PID控制器Python扩展',
    ext_modules=[pid_cpp_module]
)
