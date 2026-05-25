#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CARLA 智能录制脚本 - 带车辆状态监测

功能：
1. 实时监测所有车辆速度和碰撞状态
2. 如果车辆不动或撞墙，自动停止并重新录制
3. 最多重试3次，确保录制到有效视频
4. 自动生成详细报告

遵循项目规范：PEP8、4空格缩进、UTF-8编码
"""

import os
import sys
import io
import json
import time
import signal
from datetime import datetime
from pathlib import Path

import carla
import pygame
import cv2
import numpy as np

# Windows console encoding fix
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')


# ==================== 配置参数 ====================
CARLA_HOST = "localhost"
CARLA_PORT = 2000
CARLA_TIMEOUT = 5.0
JSON_FILE = r"C:\Users\22317\OneDrive\Desktop\zhipu_test_results\result_01.json"

DESKTOP = Path.home() / "OneDrive" / "Desktop"
TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")

# Video recording settings
RECORD_FPS = 15
TARGET_DURATION = 90  # Target 90 seconds (more reliable)
MIN_VALID_DURATION = 60  # Minimum valid recording duration
IMG_WIDTH = 800
IMG_HEIGHT = 600
MAX_RETRIES = 3  # Maximum retry attempts

# Vehicle monitoring thresholds
MIN_SPEED_THRESHOLD = 0.5  # m/s - below this considered stationary
COLLISION_IMPULSE_THRESHOLD = 1000  # Collision impulse threshold
STATIONARY_TIME_LIMIT = 15  # seconds - if all vehicles stationary for this long, restart
MIN_MOVING_VEHICLES_RATIO = 0.2  # At least 20% of vehicles should be moving


class VehicleMonitor:
    """车辆状态监测器"""

    def __init__(self, vehicles, world):
        self.vehicles = vehicles
        self.world = world
        self.collision_events = []
        self.stationary_start_time = None
        self.is_collision_detected = False

    def setup_collision_detection(self):
        """为所有车辆设置碰撞检测传感器"""
        blueprint_library = self.world.get_blueprint_library()
        collision_bp = blueprint_library.find('sensor.other.collision')

        self.collision_sensors = []
        for i, vehicle in enumerate(self.vehicles):
            try:
                sensor = self.world.spawn_actor(
                    collision_bp,
                    carla.Transform(),
                    attach_to=vehicle
                )
                sensor.listen(lambda event, vid=i: self._on_collision(event, vid))
                self.collision_sensors.append(sensor)
            except Exception as e:
                print(f"  ⚠ 车辆 {i} 碰撞传感器设置失败: {e}")

        print(f"  ✓ 已为 {len(self.collision_sensors)} 辆车设置碰撞检测")

    def _on_collision(self, event, vehicle_id):
        """碰撞事件回调"""
        impulse = event.normal_impulse
        impulse_magnitude = (impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2) ** 0.5

        if impulse_magnitude > COLLISION_IMPULSE_THRESHOLD:
            self.is_collision_detected = True
            self.collision_events.append({
                'vehicle_id': vehicle_id,
                'time': time.time(),
                'impulse': impulse_magnitude,
                'location': (event.transform.location.x,
                           event.transform.location.y,
                           event.transform.location.z)
            })
            print(f"\n  [!] 检测到碰撞! 车辆 {vehicle_id}, 冲量: {impulse_magnitude:.1f}")

    def check_vehicles_status(self):
        """检查所有车辆状态，返回是否应该继续录制"""
        moving_count = 0
        total_speed = 0
        vehicle_statuses = []

        for i, vehicle in enumerate(self.vehicles):
            try:
                velocity = vehicle.get_velocity()
                speed = velocity.length()
                total_speed += speed

                # Check if vehicle is destroyed
                if not vehicle.is_alive:
                    vehicle_statuses.append({
                        'id': i,
                        'speed': 0,
                        'status': 'destroyed'
                    })
                    continue

                if speed > MIN_SPEED_THRESHOLD:
                    moving_count += 1
                    vehicle_statuses.append({
                        'id': i,
                        'speed': speed,
                        'status': 'moving'
                    })
                else:
                    vehicle_statuses.append({
                        'id': i,
                        'speed': speed,
                        'status': 'stationary'
                    })
            except Exception as e:
                vehicle_statuses.append({
                    'id': i,
                    'speed': 0,
                    'status': f'error: {str(e)[:30]}'
                })

        total_vehicles = len(self.vehicles)
        moving_ratio = moving_count / total_vehicles if total_vehicles > 0 else 0
        avg_speed = total_speed / total_vehicles if total_vehicles > 0 else 0

        # Check if all vehicles are stationary
        if moving_count == 0 or moving_ratio < MIN_MOVING_VEHICLES_RATIO:
            if self.stationary_start_time is None:
                self.stationary_start_time = time.time()
                print(f"\n  [!] 警告: 仅 {moving_count}/{total_vehicles} 辆车在移动")
            else:
                stationary_duration = time.time() - self.stationary_start_time
                if stationary_duration > STATIONARY_TIME_LIMIT:
                    print(f"\n  [!] 所有车辆静止超过 {STATIONARY_TIME_LIMIT} 秒，需要重新录制")
                    return False, {
                        'reason': 'all_stationary',
                        'duration': stationary_duration,
                        'moving_count': moving_count,
                        'total_vehicles': total_vehicles
                    }
        else:
            self.stationary_start_time = None

        # Check for collisions
        if self.is_collision_detected:
            print(f"\n  [!] 检测到碰撞事件，需要重新录制")
            return False, {
                'reason': 'collision',
                'collision_count': len(self.collision_events),
                'moving_count': moving_count,
                'total_vehicles': total_vehicles
            }

        return True, {
            'moving_count': moving_count,
            'total_vehicles': total_vehicles,
            'avg_speed': avg_speed,
            'moving_ratio': moving_ratio
        }

    def cleanup(self):
        """清理碰撞传感器"""
        for sensor in self.collision_sensors:
            try:
                sensor.destroy()
            except:
                pass


class SmartCarlaRecorder:
    """智能 CARLA 录制器"""

    def __init__(self):
        self.client = None
        self.world = None
        self.ego_vehicle = None
        self.camera = None
        self.vehicles = []
        self.image_data = None
        self.recording_attempts = 0
        self.successful_recording = False
        self.final_video_path = None
        self.weather_str = "未知"
        self.time_str = "未知"
        self.vehicle_count = 0
        self.records = []

    def connect_carla(self):
        """连接到 CARLA 服务器"""
        print("\n" + "=" * 70)
        print("[步骤1] 连接 CARLA 服务器")
        print("=" * 70)

        try:
            self.client = carla.Client(CARLA_HOST, CARLA_PORT)
            self.client.set_timeout(CARLA_TIMEOUT)
            self.world = self.client.get_world()
            print(f"  ✓ 连接成功 - 地图: {self.world.get_map().name}")
            return True
        except Exception as e:
            print(f"  ✗ 连接失败: {e}")
            return False

    def parse_json_config(self):
        """解析 JSON 场景配置"""
        print("\n" + "=" * 70)
        print("[步骤2] 解析场景配置")
        print("=" * 70)

        try:
            with open(JSON_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)

            scene = data.get('parsed_json', data)
            self.weather_str = scene.get('weather', scene.get('天气', '下雨'))
            self.time_str = scene.get('time', scene.get('时间', '傍晚'))
            count = scene.get('vehicleCount',
                     scene.get('vehicle_count',
                     scene.get('车辆数量', 10)))
            self.vehicle_count = min(int(count), 10)

            print(f"  天气: {self.weather_str}")
            print(f"  时间: {self.time_str}")
            print(f"  车辆数量: {self.vehicle_count}")
            return True

        except Exception as e:
            print(f"  ⚠ JSON 解析失败，使用默认配置: {e}")
            self.weather_str = "下雨"
            self.time_str = "傍晚"
            self.vehicle_count = 10
            return True

    def configure_scene(self):
        """配置场景（天气+车辆）"""
        print("\n" + "=" * 70)
        print("[步骤3] 配置场景")
        print("=" * 70)

        try:
            # Configure weather
            weather = carla.WeatherParameters()
            wl = str(self.weather_str).lower()

            if any(k in wl for k in ['雨', 'rain']):
                weather.cloudiness = 80.0
                weather.precipitation = 60.0
                weather.precipitation_deposits = 50.0
                weather.wind_intensity = 50.0
                weather.fog_density = 20.0
                print("  ✓ 天气: 雨天模式")
            else:
                weather.cloudiness = 30.0
                print("  ✓ 天气: 默认")

            tl = str(self.time_str).lower()
            if any(k in tl for k in ['傍晚', 'evening', 'dusk', '黄昏']):
                weather.sun_altitude_angle = 10.0
                weather.sun_azimuth_angle = 200.0
                print("  ✓ 时间: 傍晚")
            else:
                weather.sun_altitude_angle = 45.0

            self.world.set_weather(weather)

            # Spawn ego vehicle
            blueprint_library = self.world.get_blueprint_library()
            spawn_points = self.world.get_map().get_spawn_points()

            ego_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

            # Try multiple spawn points for ego
            for sp_idx in range(min(5, len(spawn_points))):
                self.ego_vehicle = self.world.try_spawn_actor(ego_bp, spawn_points[sp_idx])
                if self.ego_vehicle:
                    print(f"  ✓ 主角车辆已生成 (位置 {sp_idx})")
                    break

            if not self.ego_vehicle:
                print("  ⚠ 主角车辆生成失败，将使用第一辆自动驾驶车辆安装摄像头")

            # Install RGB camera - try ego first, then fall back to first autopilot vehicle
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', str(IMG_WIDTH))
            camera_bp.set_attribute('image_size_y', str(IMG_HEIGHT))
            camera_bp.set_attribute('fov', '90')

            camera_transform = carla.Transform(
                carla.Location(x=2.0, z=1.4),
                carla.Rotation(pitch=0)
            )

            def camera_callback(image):
                image.convert(carla.ColorConverter.Raw)
                arr = np.frombuffer(image.raw_data, dtype=np.uint8)
                arr = arr.reshape((image.height, image.width, 4))
                arr = arr[:, :, :3]
                arr = arr[:, :, ::-1]
                self.image_data = arr

            if self.ego_vehicle:
                self.camera = self.world.spawn_actor(
                    camera_bp, camera_transform,
                    attach_to=self.ego_vehicle
                )
                self.camera.listen(camera_callback)
                print("  ✓ RGB 摄像头已安装 (主角车辆)")
            else:
                # Will install on first autopilot vehicle after spawning
                print("  ⚠ 将在第一辆自动驾驶车辆上安装摄像头")

            # Configure TrafficManager
            tm = self.client.get_trafficmanager(CARLA_PORT)
            tm.set_synchronous_mode(False)
            tm.set_hybrid_physics_mode(True)
            tm.set_global_distance_to_leading_vehicle(5.0)
            print("  ✓ TrafficManager 已配置")

            # Spawn autopilot vehicles
            vehicle_bps = [
                bp for bp in blueprint_library.filter('vehicle.*')
                if bp.has_attribute('number_of_wheels') and
                int(bp.get_attribute('number_of_wheels').as_int()) == 4
            ]
            vehicle_bps = [bp for bp in vehicle_bps
                          if 'carlamotors' not in bp.id and 'firetruck' not in bp.id
                          and 'ambulance' not in bp.id and 'bus' not in bp.id.lower()]

            print(f"  找到 {len(vehicle_bps)} 种可用车辆蓝图")

            used_positions = []
            if self.ego_vehicle:
                ego_loc = self.ego_vehicle.get_transform().location
                used_positions.append((ego_loc.x, ego_loc.y, ego_loc.z))

            spawned = 0
            valid_spawns = []
            for sp in spawn_points:
                if len(valid_spawns) >= self.vehicle_count * 3:
                    break
                too_close = False
                for pos in used_positions:
                    dx = sp.location.x - pos[0]
                    dy = sp.location.y - pos[1]
                    dist = (dx*dx + dy*dy) ** 0.5
                    if dist < 8.0:
                        too_close = True
                        break
                if not too_close:
                    valid_spawns.append(sp)

            for sp in valid_spawns:
                if spawned >= self.vehicle_count:
                    break

                try:
                    bp = vehicle_bps[spawned % len(vehicle_bps)]
                    if bp.has_attribute('color'):
                        colors = bp.get_attribute('color').recommended_values
                        bp.set_attribute('color', colors[spawned % len(colors)])

                    vehicle = self.world.try_spawn_actor(bp, sp)
                    if vehicle:
                        vehicle.set_autopilot(True)
                        try:
                            tm.ignore_lights_percentage(vehicle, 0)
                            tm.ignore_signs_percentage(vehicle, 0)
                            tm.distance_to_leading_vehicle(vehicle, 5.0)
                        except:
                            pass
                        self.vehicles.append(vehicle)
                        used_positions.append((sp.location.x, sp.location.y, sp.location.z))

                        # Install camera on first vehicle if ego failed
                        if not self.ego_vehicle and not self.camera and spawned == 0:
                            self.camera = self.world.spawn_actor(
                                camera_bp, camera_transform,
                                attach_to=vehicle
                            )
                            self.camera.listen(camera_callback)
                            print(f"  ✓ RGB 摄像头已安装 (自动驾驶车辆 {spawned + 1})")

                        spawned += 1
                except Exception:
                    pass

            # Initialize physics
            print("\n  初始化物理引擎...")
            for _ in range(10):
                self.world.tick()
                time.sleep(0.2)

            time.sleep(3)

            # Check initial movement
            moving = sum(1 for v in self.vehicles if v.get_velocity().length() > 0.5)
            print(f"  ✓ 场景配置完成！共 {spawned} 辆车，{moving} 辆正在移动")

            return True

        except Exception as e:
            print(f"  ✗ 场景配置失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def record_with_monitoring(self, attempt_num):
        """带监测的录制"""
        print("\n" + "=" * 70)
        print(f"[步骤4] 录制尝试 {attempt_num + 1}/{MAX_RETRIES}")
        print("=" * 70)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_path = str(DESKTOP / f"carla_smart_{timestamp}.mp4")

        try:
            # Initialize pygame
            pygame.init()
            screen = pygame.display.set_mode((IMG_WIDTH, IMG_HEIGHT))
            pygame.display.set_caption(f"CARLA Smart Record - Attempt {attempt_num + 1}")
            clock = pygame.time.Clock()

            # Video writer
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(video_path, fourcc, RECORD_FPS, (IMG_WIDTH, IMG_HEIGHT))

            if not writer.isOpened():
                print("  ✗ 视频编码器失败")
                pygame.quit()
                return False, {'error': 'video_writer_failed'}

            # Setup vehicle monitor
            monitor = VehicleMonitor(self.vehicles, self.world)
            monitor.setup_collision_detection()

            print(f"  开始录制... (目标: {TARGET_DURATION}秒)")
            print(f"  监测: 车速 < {MIN_SPEED_THRESHOLD}m/s 视为静止")
            print(f"  监测: 碰撞冲量 > {COLLISION_IMPULSE_THRESHOLD}")
            print(f"  自动重启: 静止超过 {STATIONARY_TIME_LIMIT}秒 或 发生碰撞\n")

            start_time = time.time()
            frame_count = 0
            prev_image = None
            last_status_print = 0

            while True:
                elapsed = time.time() - start_time

                # Check if reached target duration
                if elapsed >= TARGET_DURATION:
                    print(f"\n  ✓ 达到目标时长 {TARGET_DURATION}秒")
                    break

                # Handle pygame events
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        break
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            print("\n  用户按 ESC 退出")
                            break

                # Get camera image
                if self.image_data is not None:
                    surface = pygame.surfarray.make_surface(
                        self.image_data.swapaxes(0, 1)
                    )
                    prev_image = surface
                elif prev_image is not None:
                    surface = prev_image
                else:
                    screen.fill((30, 30, 30))
                    font = pygame.font.Font(None, 36)
                    text = font.render("Waiting for camera...", True, (255, 255, 255))
                    screen.blit(text, (250, 280))
                    pygame.display.flip()
                    clock.tick(RECORD_FPS)
                    continue

                # Display
                screen.blit(surface, (0, 0))

                # Monitor vehicles every second
                if elapsed - last_status_print >= 1.0:
                    should_continue, status = monitor.check_vehicles_status()

                    if not should_continue:
                        reason = status.get('reason', 'unknown')
                        print(f"\n  [!] 录制中断原因: {reason}")
                        print(f"  已录制: {elapsed:.1f}秒")

                        # Cleanup and mark for retry
                        writer.release()
                        pygame.quit()
                        monitor.cleanup()

                        # Delete failed video
                        if os.path.exists(video_path):
                            os.remove(video_path)
                            print(f"  已删除无效视频: {video_path}")

                        return False, {
                            'reason': reason,
                            'duration': elapsed,
                            'status': status
                        }

                    # Print status
                    if status.get('moving_count') is not None:
                        print(f"  状态: {status['moving_count']}/{status['total_vehicles']} 辆移动 | "
                              f"平均速度: {status.get('avg_speed', 0):.2f} m/s | "
                              f"时长: {elapsed:.0f}/{TARGET_DURATION}秒")

                    last_status_print = elapsed

                # HUD overlay
                font = pygame.font.Font(None, 24)
                hud_text = (
                    f"CARLA Smart Record | {self.weather_str} {self.time_str} | "
                    f"{int(elapsed)}/{TARGET_DURATION}s | Attempt {attempt_num + 1}"
                )
                overlay = pygame.Surface((IMG_WIDTH, 30))
                overlay.set_alpha(180)
                overlay.fill((0, 0, 0))
                screen.blit(overlay, (0, 0))
                text_surf = font.render(hud_text, True, (255, 255, 255))
                screen.blit(text_surf, (10, 5))
                pygame.display.flip()

                # Write frame
                if self.image_data is not None:
                    bgr_frame = self.image_data[:, :, ::-1].copy()
                    writer.write(bgr_frame)
                    frame_count += 1

                clock.tick(RECORD_FPS)
                pygame.event.pump()

            # Successful recording
            writer.release()
            pygame.quit()
            monitor.cleanup()

            duration = time.time() - start_time
            print(f"\n  ✓ 录制成功!")
            print(f"    时长: {duration:.1f}秒")
            print(f"    帧数: {frame_count}")
            print(f"    文件: {video_path}")

            return True, {
                'video_path': video_path,
                'duration': duration,
                'frames': frame_count
            }

        except Exception as e:
            print(f"\n  ✗ 录制出错: {e}")
            import traceback
            traceback.print_exc()
            try:
                pygame.quit()
            except:
                pass
            return False, {'error': str(e)}

    def cleanup_all(self):
        """清理所有资源"""
        print("\n" + "=" * 70)
        print("[清理] 释放资源")
        print("=" * 70)

        try:
            if self.camera:
                self.camera.stop()
                self.camera.destroy()
                print("  ✓ 摄像头已销毁")

            if self.vehicles:
                for v in self.vehicles:
                    try:
                        v.destroy()
                    except:
                        pass
                print(f"  ✓ {len(self.vehicles)} 辆车已清理")

            if self.ego_vehicle:
                try:
                    self.ego_vehicle.destroy()
                except:
                    pass
                print("  ✓ 主角车辆已清理")

        except Exception as e:
            print(f"  清理警告: {e}")

    def generate_report(self):
        """生成执行报告"""
        print("\n" + "=" * 70)
        print("[报告] 生成执行报告")
        print("=" * 70)

        report_path = str(DESKTOP / "smart_record_report.txt")

        report = f"""{'='*70}
CARLA 智能录制报告
{'='*70}

执行时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
总尝试次数: {self.recording_attempts}
最终状态: {'成功' if self.successful_recording else '失败'}

--- 场景配置 ---
天气: {self.weather_str}
时间: {self.time_str}
车辆数量: {len(self.vehicles)}
地图: {self.world.get_map().name if self.world else 'N/A'}

--- 录制记录 ---
"""
        for i, record in enumerate(self.records, 1):
            report += f"\n尝试 {i}:\n"
            report += f"  结果: {record.get('result', 'unknown')}\n"
            report += f"  时长: {record.get('duration', 0):.1f}秒\n"
            if record.get('reason'):
                report += f"  中断原因: {record['reason']}\n"
            if record.get('video_path'):
                size_mb = os.path.getsize(record['video_path']) / (1024 * 1024) if os.path.exists(record['video_path']) else 0
                report += f"  视频文件: {record['video_path']}\n"
                report += f"  文件大小: {size_mb:.1f} MB\n"

        if self.final_video_path:
            report += f"\n--- 最终视频 ---\n"
            report += f"文件路径: {self.final_video_path}\n"
            if os.path.exists(self.final_video_path):
                size_mb = os.path.getsize(self.final_video_path) / (1024 * 1024)
                report += f"文件大小: {size_mb:.1f} MB\n"

        report += f"\n{'='*70}\n"

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(report)

        print(f"  报告已保存: {report_path}")
        print(report)

    def run(self):
        """主执行流程"""
        print("\n" + "#" * 70)
        print("# CARLA 智能录制系统")
        print("# 自动监测车辆状态，失败自动重试")
        print("#" * 70)

        try:
            # Step 1: Connect
            if not self.connect_carla():
                return False

            # Step 2: Parse config
            self.parse_json_config()

            # Step 3: Configure scene
            if not self.configure_scene():
                return False

            # Step 4: Record with retries
            for attempt in range(MAX_RETRIES):
                self.recording_attempts = attempt + 1
                success, result = self.record_with_monitoring(attempt)

                if success:
                    self.successful_recording = True
                    self.final_video_path = result['video_path']
                    self.records.append({
                        'result': 'success',
                        'duration': result['duration'],
                        'frames': result['frames'],
                        'video_path': result['video_path']
                    })
                    break
                else:
                    self.records.append({
                        'result': 'failed',
                        'duration': result.get('duration', 0),
                        'reason': result.get('reason', 'unknown'),
                        'status': result.get('status', {})
                    })

                    if attempt < MAX_RETRIES - 1:
                        print(f"\n  >>> 准备第 {attempt + 2} 次重试...")
                        # Clean up old vehicles
                        for v in self.vehicles:
                            try:
                                v.destroy()
                            except:
                                pass
                        self.vehicles = []

                        # Re-spawn vehicles
                        print("  重新生成车辆...")
                        if not self.configure_scene():
                            break
                        time.sleep(2)
                    else:
                        print(f"\n  ✗ 已达到最大重试次数 ({MAX_RETRIES})")

            # Step 5: Cleanup
            self.cleanup_all()

            # Step 6: Report
            self.generate_report()

            print("\n" + "#" * 70)
            if self.successful_recording:
                print("# 录制成功完成！")
            else:
                print("# 录制失败，请检查 CARLA 环境")
            print("#" * 70)

            return self.successful_recording

        except KeyboardInterrupt:
            print("\n\n  用户中断")
            self.cleanup_all()
            return False

        except Exception as e:
            print(f"\n✗ 未捕获异常: {e}")
            import traceback
            traceback.print_exc()
            self.cleanup_all()
            return False


if __name__ == "__main__":
    recorder = SmartCarlaRecorder()
    result = recorder.run()
    sys.exit(0 if result else 1)
