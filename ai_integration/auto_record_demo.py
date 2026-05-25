#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CARLA AI场景自动演示视频录制脚本

功能：
1. 检查并启动CARLA服务器
2. 运行carla_ai_scene.py自动配置场景
3. 使用mss+OpenCV录制CARLA窗口
4. 生成执行报告

作者: AI Assistant
日期: 2026-04-13
"""

import os
import sys
import io
import time
import json
import subprocess
import threading
import signal
from datetime import datetime
from pathlib import Path

# 设置控制台输出编码为UTF-8
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

import carla
import cv2
import numpy as np
import mss
import pygetwindow as gw

# ==================== 配置参数 ====================
CARLA_SERVER_PATH = r"D:\Carla_0.9.13\CarlaUE4.exe"
CARLA_ARGS = ["-quality-level=Low", "-ResX=1280", "-ResY=720", "-windowed"]
PROJECT_DIR = r"d:\项目一"
CARLA_SCRIPT = os.path.join(PROJECT_DIR, "carla_ai_scene.py")
JSON_FILE = r"C:\Users\22317\OneDrive\Desktop\zhipu_test_results\result_01.json"
CARLA_HOST = "localhost"
CARLA_PORT = 2000
CARLA_TIMEOUT = 5.0

DESKTOP = Path.home() / "OneDrive" / "Desktop"
TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
VIDEO_FILENAME = f"carla_ai_demo_{TIMESTAMP}.mp4"
VIDEO_PATH = DESKTOP / VIDEO_FILENAME
REPORT_PATH = DESKTOP / "report.txt"

RECORD_FPS = 15
RECORD_DURATION = 120  # 录制2分钟

class CarlaDemoRecorder:
    """CARLA演示录制器"""

    def __init__(self):
        self.report = {
            "start_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "status": "success",
            "video_path": "",
            "record_duration": 0,
            "carla_server_started": False,
            "scene_configured": False,
            "recording_success": False,
            "issues": [],
            "fixes": []
        }

    def log_issue(self, issue, fix=""):
        """记录问题及修复措施"""
        self.report["issues"].append(issue)
        if fix:
            self.report["fixes"].append(fix)
        print(f"  [!] 问题: {issue}")
        if fix:
            print(f"  [~] 修复: {fix}")

    def check_carla_server(self):
        """检查CARLA服务器是否运行"""
        print("\n" + "=" * 70)
        print("[步骤1] 检查CARLA服务器状态")
        print("=" * 70)

        try:
            result = subprocess.run(
                ["powershell", "-Command",
                 "Get-Process -Name 'CarlaUE4*' -ErrorAction SilentlyContinue"],
                capture_output=True, text=True, timeout=5
            )
            if "CarlaUE4" in result.stdout:
                print("  ✓ CARLA服务器已在运行")
                return True
            else:
                print("  ✗ CARLA服务器未运行")
                return False
        except Exception as e:
            self.log_issue(f"检查CARLA进程失败: {e}")
            return False

    def start_carla_server(self):
        """启动CARLA服务器"""
        print("\n" + "=" * 70)
        print("[步骤2] 启动CARLA服务器")
        print("=" * 70)

        if not os.path.exists(CARLA_SERVER_PATH):
            # 尝试备用路径
            alt_path = r"D:\Carla_0.9.13\WindowsNoEditor\CarlaUE4.exe"
            if os.path.exists(alt_path):
                self.log_issue(
                    f"CARLA主路径不存在，使用备用路径: {alt_path}",
                    f"找到备用可执行文件"
                )
                server_path = alt_path
            else:
                self.report["status"] = "failed"
                self.log_issue(f"CARLA服务器可执行文件不存在: {CARLA_SERVER_PATH}")
                return False
        else:
            server_path = CARLA_SERVER_PATH

        try:
            # 使用DETACHED_PROCESS让进程在后台运行
            startupinfo = subprocess.STARTUPINFO()
            startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            startupinfo.wShowWindow = 1  # SW_SHOWNORMAL

            args = [server_path] + CARLA_ARGS
            print(f"  启动命令: {server_path} {' '.join(CARLA_ARGS)}")

            self.carla_process = subprocess.Popen(
                args,
                startupinfo=startupinfo,
                creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
            )
            print(f"  ✓ CARLA服务器进程已启动 (PID: {self.carla_process.pid})")
            self.report["carla_server_started"] = True

            # 等待服务器初始化
            print(f"  等待服务器初始化 (15秒)...")
            time.sleep(15)

            # 验证连接
            try:
                client = carla.Client(CARLA_HOST, CARLA_PORT)
                client.set_timeout(10.0)
                world = client.get_world()
                print(f"  ✓ 服务器已就绪 - 地图: {world.get_map().name}")
                return True
            except Exception as e:
                self.log_issue(
                    f"CARLA服务器启动后连接失败: {e}",
                    "等待更长时间后重试"
                )
                time.sleep(10)
                try:
                    client = carla.Client(CARLA_HOST, CARLA_PORT)
                    client.set_timeout(10.0)
                    world = client.get_world()
                    print(f"  ✓ 服务器已就绪（重试后）- 地图: {world.get_map().name}")
                    return True
                except:
                    self.report["status"] = "failed"
                    self.log_issue("CARLA服务器启动失败，无法连接")
                    return False

        except Exception as e:
            self.report["status"] = "failed"
            self.log_issue(f"启动CARLA服务器失败: {e}")
            return False

    def find_carla_window(self):
        """查找CARLA窗口"""
        print("\n" + "=" * 70)
        print("[步骤3] 查找CARLA窗口")
        print("=" * 70)

        try:
            # 尝试多种窗口标题匹配
            possible_titles = ["CARLA", "Unreal", "CarlaUE4"]

            for title_keyword in possible_titles:
                try:
                    windows = gw.getWindowsWithTitle(title_keyword)
                    for win in windows:
                        if win.width > 100 and win.height > 100:  # 过滤小窗口
                            print(f"  ✓ 找到CARLA窗口: {win.title}")
                            print(f"    位置: ({win.left}, {win.top}) 大小: {win.width}x{win.height}")
                            return win
                except:
                    continue

            # 如果没找到，列出所有窗口
            all_windows = gw.getAllTitles()
            carla_like = [t for t in all_windows if any(k in t for k in possible_titles)]
            if carla_like:
                print(f"  找到疑似CARLA窗口: {carla_like}")
            else:
                self.log_issue("未找到CARLA窗口，将使用全屏幕录制")

            return None

        except Exception as e:
            self.log_issue(f"查找窗口失败: {e}", "将使用屏幕区域录制")
            return None

    def run_carla_scene_script(self):
        """运行carla_ai_scene.py脚本"""
        print("\n" + "=" * 70)
        print("[步骤4] 运行CARLA场景配置脚本")
        print("=" * 70)

        python_exe = r"D:\conda\envs\carla_env\python.exe"

        # 修改脚本使其在配置完成后自动退出（而不是无限循环等待）
        # 我们先连接到CARLA，直接配置场景，不依赖外部脚本
        print("  直接通过Python API配置场景...")

        try:
            client = carla.Client(CARLA_HOST, CARLA_PORT)
            client.set_timeout(CARLA_TIMEOUT)
            world = client.get_world()

            # 解析JSON
            print(f"  读取JSON: {JSON_FILE}")
            with open(JSON_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
            scene_data = data.get('parsed_json', data)

            weather_str = scene_data.get('weather', scene_data.get('天气', ''))
            time_str = scene_data.get('time', scene_data.get('时间', ''))
            vehicle_count = scene_data.get('vehicleCount',
                           scene_data.get('vehicle_count',
                           scene_data.get('车辆数量', 10)))
            vehicle_count = int(vehicle_count)

            print(f"  天气: {weather_str}, 时间: {time_str}, 车辆: {vehicle_count}")

            # 配置天气
            weather = carla.WeatherParameters()
            weather_lower = str(weather_str).lower()

            if any(k in weather_lower for k in ['雨', 'rain']):
                weather.cloudiness = 80.0
                weather.precipitation = 60.0
                weather.precipitation_deposits = 50.0
                weather.wind_intensity = 50.0
                print("  ✓ 设置为雨天")
            else:
                weather.cloudiness = 30.0
                print("  ✓ 使用默认天气")

            time_lower = str(time_str).lower()
            if any(k in time_lower for k in ['傍晚', 'evening', 'dusk']):
                weather.sun_altitude_angle = 10.0
                weather.sun_azimuth_angle = 200.0
                print("  ✓ 设置为傍晚时间")
            else:
                weather.sun_altitude_angle = 45.0

            world.set_weather(weather)
            print("  ✓ 天气已应用")

            # 生成车辆（使用TrafficManager）
            tm = client.get_trafficmanager(CARLA_PORT)
            tm.set_synchronous_mode(False)
            tm.set_hybrid_physics_mode(True)  # Enable hybrid physics for better performance
            tm.set_global_distance_to_leading_vehicle(5.0)  # Safe following distance
            print("  ✓ TrafficManager 已配置（混合物理模式+安全车距）")

            blueprint_library = world.get_blueprint_library()
            vehicle_blueprints = [
                bp for bp in blueprint_library.filter('vehicle.*')
                if bp.has_attribute('number_of_wheels') and
                int(bp.get_attribute('number_of_wheels').as_int()) == 4
            ]

            # 限制车辆数量不超过12辆
            vehicle_count = min(vehicle_count, 12)

            spawn_points = world.get_map().get_spawn_points()
            selected_spawns = []
            used_positions = []

            for spawn in spawn_points:
                if len(selected_spawns) >= vehicle_count:
                    break
                # 检查2米内是否有其他车辆
                too_close = False
                for pos in used_positions:
                    dist = ((spawn.location.x - pos[0]) ** 2 +
                            (spawn.location.y - pos[1]) ** 2 +
                            (spawn.location.z - pos[2]) ** 2) ** 0.5
                    if dist < 2.0:
                        too_close = True
                        break
                if not too_close:
                    selected_spawns.append(spawn)
                    used_positions.append((spawn.location.x, spawn.location.y, spawn.location.z))

            vehicles = []
            for i, spawn_point in enumerate(selected_spawns):
                try:
                    blueprint = vehicle_blueprints[i % len(vehicle_blueprints)]
                    if blueprint.has_attribute('color'):
                        color = blueprint.get_attribute('color').recommended_values[
                            i % len(blueprint.get_attribute('color').recommended_values)]
                        blueprint.set_attribute('color', color)

                    vehicle = world.try_spawn_actor(blueprint, spawn_point)
                    if vehicle:
                        vehicle.set_autopilot(True)
                        # Configure TrafficManager parameters for this vehicle (CARLA 0.9.13 compatible)
                        try:
                            tm.ignore_lights_percentage(vehicle, 0)  # Obey traffic lights
                            tm.ignore_signs_percentage(vehicle, 0)   # Obey traffic signs
                            tm.distance_to_leading_vehicle(vehicle, 5.0)  # Safe following distance
                        except Exception:
                            pass  # TM settings are optional
                        vehicles.append(vehicle)
                        print(f"  ✓ 车辆 {len(vehicles)}/{vehicle_count} 已生成: {blueprint.id.split('/')[-1]} (autopilot enabled)")
                except Exception as e:
                    print(f"  ⚠ 车辆 {i+1} 生成失败: {e}")

            # Force world ticks to initialize physics for all vehicles
            print("\n  初始化车辆物理引擎...")
            import time as time_module
            for _ in range(10):
                world.tick()
                time_module.sleep(0.2)

            # Give TrafficManager more time to fully initialize and assign routes
            print("  等待 TrafficManager 分配路线...")
            time_module.sleep(3)

            # Verify vehicles are moving and trigger movement if needed
            moving_count = 0
            stationary_vehicles = []
            for v in vehicles:
                speed = v.get_velocity()
                if speed.length() > 0.1:
                    moving_count += 1
                else:
                    stationary_vehicles.append(v)

            # If some vehicles are not moving, apply manual impulse to start them
            if stationary_vehicles:
                print(f"  检测到 {len(stationary_vehicles)} 辆静止车辆，尝试手动启动...")
                for v in stationary_vehicles:
                    try:
                        # Apply small forward impulse to kickstart the vehicle
                        transform = v.get_transform()
                        forward_vector = transform.get_forward_vector()
                        impulse = forward_vector * 50000  # Strong enough to start movement
                        v.add_impulse(impulse)
                    except Exception as e:
                        pass

                # Wait for vehicles to respond to impulses
                time_module.sleep(2)
                for _ in range(5):
                    world.tick()
                    time_module.sleep(0.2)

                # Re-check movement
                moving_count = 0
                for v in vehicles:
                    speed = v.get_velocity()
                    if speed.length() > 0.1:
                        moving_count += 1

            print(f"\n  ✓ 场景配置完成！共生成 {len(vehicles)} 辆车")
            print(f"  正在移动的车辆: {moving_count}/{len(vehicles)}")

            self.report["scene_configured"] = True
            self.vehicles = vehicles
            self.client = client
            self.world = world
            self.tm = tm

            return True

        except Exception as e:
            self.report["status"] = "failed"
            self.log_issue(f"场景配置失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def record_screen(self, carla_window=None):
        """录制屏幕"""
        print("\n" + "=" * 70)
        print("[步骤5] 开始屏幕录制")
        print("=" * 70)

        try:
            # 确定录制区域
            if carla_window and carla_window.left >= 0 and carla_window.top >= 0:
                monitor = {
                    "top": carla_window.top,
                    "left": carla_window.left,
                    "width": carla_window.width,
                    "height": carla_window.height
                }
                print(f"  录制CARLA窗口区域: {monitor}")
            else:
                # 使用固定区域（假设CARLA窗口在屏幕中央）
                monitor = {"top": 0, "left": 0, "width": 1280, "height": 720}
                self.log_issue("使用固定录制区域", "CARLA窗口未检测到")
                print(f"  使用固定区域: {monitor}")

            # 确保窗口在前台
            if carla_window:
                try:
                    carla_window.activate()
                    time.sleep(1)
                    print("  ✓ CARLA窗口已激活到前台")
                except:
                    pass

            # 设置视频写入器
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(
                str(VIDEO_PATH),
                fourcc,
                RECORD_FPS,
                (monitor['width'], monitor['height'])
            )

            if not out.isOpened():
                self.log_issue("VideoWriter初始化失败", "尝试其他编码器")
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                out = cv2.VideoWriter(
                    str(VIDEO_PATH).replace('.mp4', '.avi'),
                    fourcc,
                    RECORD_FPS,
                    (monitor['width'], monitor['height'])
                )

            print(f"  开始录制... (FPS: {RECORD_FPS}, 时长: {RECORD_DURATION}秒)")

            start_time = time.time()
            frame_count = 0
            error_count = 0
            max_errors = 50  # 允许一定数量的连续错误

            with mss.mss() as sct:
                while time.time() - start_time < RECORD_DURATION:
                    try:
                        # 捕获屏幕
                        screenshot = sct.grab(monitor)
                        frame = np.array(screenshot)
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                        # 写入视频
                        out.write(frame)
                        frame_count += 1

                        # 每30帧显示进度
                        if frame_count % 30 == 0:
                            elapsed = time.time() - start_time
                            print(f"  录制中... {elapsed:.0f}/{RECORD_DURATION}秒 "
                                  f"({frame_count}帧)")

                        # 泵送事件防止阻塞
                        error_count = 0

                    except Exception as e:
                        error_count += 1
                        if error_count > max_errors:
                            self.log_issue(
                                f"连续录制错误过多: {error_count}",
                                "停止录制并保存已有内容"
                            )
                            break
                        time.sleep(0.1)

            # 释放资源
            out.release()
            record_duration = time.time() - start_time
            self.report["record_duration"] = record_duration

            print(f"\n  ✓ 录制完成!")
            print(f"    时长: {record_duration:.1f}秒")
            print(f"    帧数: {frame_count}")
            print(f"    文件: {VIDEO_PATH}")

            self.report["recording_success"] = True
            self.report["video_path"] = str(VIDEO_PATH)

            return True

        except Exception as e:
            self.log_issue(f"录制失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def cleanup(self):
        """清理资源"""
        print("\n" + "=" * 70)
        print("[步骤6] 清理资源")
        print("=" * 70)

        try:
            if hasattr(self, 'vehicles') and self.vehicles:
                print(f"  清理 {len(self.vehicles)} 辆车辆...")
                for vehicle in self.vehicles:
                    try:
                        vehicle.destroy()
                    except:
                        pass
                print("  ✓ 车辆已清理")

            print("  ✓ 清理完成")
        except Exception as e:
            print(f"  清理过程出错: {e}")

    def generate_report(self):
        """生成执行报告"""
        print("\n" + "=" * 70)
        print("[步骤7] 生成执行报告")
        print("=" * 70)

        self.report["end_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        report_text = f"""{'='*70}
CARLA AI场景演示视频录制 - 执行报告
{'='*70}

执行时间: {self.report['start_time']}
结束时间: {self.report['end_time']}
执行状态: {'成功' if self.report['status'] == 'success' else '失败'}

--- 配置信息 ---
CARLA服务器: {'已启动' if self.report['carla_server_started'] else '已存在/未启动'}
场景配置: {'成功' if self.report['scene_configured'] else '失败'}
录制状态: {'成功' if self.report['recording_success'] else '失败'}

--- 输出文件 ---
视频文件: {self.report.get('video_path', '未生成')}
录制时长: {self.report.get('record_duration', 0):.1f}秒

--- 问题记录 ---
"""
        if self.report["issues"]:
            for i, (issue, fix) in enumerate(
                zip(self.report["issues"], self.report["fixes"]), 1
            ):
                report_text += f"{i}. {issue}\n"
                if fix:
                    report_text += f"   修复: {fix}\n"
        else:
            report_text += "无问题\n"

        report_text += f"\n{'='*70}\n"

        with open(REPORT_PATH, 'w', encoding='utf-8') as f:
            f.write(report_text)

        print(f"  报告已保存: {REPORT_PATH}")
        print(report_text)

    def run(self):
        """主执行流程"""
        print("\n" + "#" * 70)
        print("# CARLA AI场景自动演示录制")
        print("#" * 70)

        try:
            # 步骤1: 检查CARLA服务器
            if not self.check_carla_server():
                if not self.start_carla_server():
                    self.generate_report()
                    return False

            # 步骤2: 查找CARLA窗口
            carla_window = self.find_carla_window()

            # 步骤3: 配置场景
            if not self.run_carla_scene_script():
                self.generate_report()
                return False

            # 步骤4: 开始录制（先启动录制，场景已在步骤3配置好）
            print("\n  等待3秒让车辆开始行驶...")
            time.sleep(3)

            if not self.record_screen(carla_window):
                self.cleanup()
                self.generate_report()
                return False

            # 步骤5: 清理
            self.cleanup()

            # 步骤6: 生成报告
            self.generate_report()

            print("\n" + "#" * 70)
            print("# 全部完成！")
            print("#" * 70)
            print(f"\n视频文件: {VIDEO_PATH}")
            print(f"执行报告: {REPORT_PATH}")

            return True

        except KeyboardInterrupt:
            print("\n\n用户中断，正在清理...")
            self.cleanup()
            self.report["status"] = "interrupted"
            self.generate_report()
            return False

        except Exception as e:
            self.report["status"] = "failed"
            self.log_issue(f"执行过程出错: {e}")
            import traceback
            traceback.print_exc()
            self.cleanup()
            self.generate_report()
            return False


if __name__ == "__main__":
    recorder = CarlaDemoRecorder()
    success = recorder.run()
    sys.exit(0 if success else 1)
