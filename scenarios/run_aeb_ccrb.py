# -*- coding: utf-8 -*-
"""包装器：使用子进程方式执行并捕获所有输出"""
import subprocess
import sys

SCRIPT = r'D:\Carla_0.9.13\PythonAPI\examples\aeb_validation_test.py'
LOG_FILE = r'C:\Users\22317\OneDrive\Desktop\aeb_ccrb_run.log'

print("[WRAPPER] Starting execution...", flush=True)

proc = subprocess.Popen(
    [r'D:\conda\envs\carla_env\python.exe', SCRIPT],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    universal_newlines=True,
    encoding='utf-8',
    errors='replace'
)

with open(LOG_FILE, 'w', encoding='utf-8') as f:
    for line in proc.stdout:
        print(line, end='', flush=True)
        f.write(line)

print(f"\n[WRAPPER] Done. Exit code: {proc.returncode}", flush=True)
