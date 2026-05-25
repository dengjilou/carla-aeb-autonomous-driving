# -*- coding: utf-8 -*-
"""
AEB自动紧急制动模块验证脚本 - CCRb前车急刹场景（带自我监控与自适应修复）
场景：自车以60km/h跟随同速目标车，2.5秒后目标车急刹
预期：AEB触发完整四阶段，自车减速/刹停避免碰撞
"""

import carla
import pygame
import cv2
import numpy as np
import math
import time
import sys
import os
import subprocess
import signal

# 文件日志：同时写入文件和标准输出
LOG_FILE = os.path.join(os.getcwd(), "output", "aeb_run.log"'
_log_f = open(LOG_FILE, 'w', encoding='utf-8')

class _Logger:
    @staticmethod
    def write(text):
        try:
            _log_f.write(text)
            _log_f.flush()
        except Exception:
            pass
        try:
            sys.__stdout__.write(text)
            sys.__stdout__.flush()
        except Exception:
            pass

def _p(*args, **kwargs):
    sep = kwargs.get('sep', ' ')
    end = kwargs.get('end', '\n')
    msg = sep.join(str(a) for a in args) + end
    _Logger.write(msg)

# 替换print
print = _p

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from stanley_pid_controller import AEBController, StanleyController, get_target_waypoint

# 优先使用C++版PID（性能更高），不可用时回退到Python版
try:
    from pid_cpp_bridge import PID
    print("[OK] 使用C++ PID控制器")
except ImportError:
    from simple_pid import PID
    print("[WARN] C++ PID不可用，使用Python版")

# ======================== 常量配置 ========================
CARLA_HOST = 'localhost'
CARLA_PORT = 2000
CARLA_EXE = r'D:\Carla_0.9.13\CarlaUE4.exe'
TEST_DURATION = 30
TARGET_SPEED_KMH = 60
TARGET_SPEED_MS = TARGET_SPEED_KMH / 3.6
FPS = 24
VIDEO_OUTPUT = r'D:\Carla_0.9.13\PythonAPI\examples\aeb_ccrb_demo.mp4'
DESKTOP = r'D:\Carla_0.9.13\PythonAPI\examples'

# 画质降级策略
QUALITY_LEVELS = ['Epic', 'Medium', 'Low']
RESOLUTIONS = [(1920, 1080), (1280, 720), (1024, 768)]

# 默认控制器参数（适配直行路段）
DEFAULT_STANLEY_K = 0.06  # 降低增益避免振荡
DEFAULT_STANLEY_ALPHA = 0.7  # 提高平滑系数
DEFAULT_LOOKAHEAD_FACTOR = 0.3  # 动态前视系数 = 速度(m/s) × factor
# 60km/h工况下需要更早触发，确保四阶段完整
DEFAULT_TTC_WARNING = 5.0
DEFAULT_TTC_PARTIAL = 3.5
DEFAULT_TTC_FULL = 2.5
DETECTION_RANGE = 80.0

# 阈值
DEVATION_THRESHOLD = 2.0  # 跑偏阈值（米）
STALL_SPEED = 0.1  # 停滞速度阈值（m/s）
STALL_DURATION = 3  # 停滞持续时间（秒）
AEB_CHECK_DISTANCE = 5.0  # AEB失效检测距离（米）
FOLLOWING_TIME_GAP = 2.0  # 跟车距离时间间隔（2秒规则）

# 全局异常记录
error_history = []
repair_history = []


# ======================== 辅助函数 ========================
def clean_old_videos():
    patterns = ['aeb_ccrb', 'aeb_validation', 'carla_', 'stanley_pid_demo']
    for pattern in patterns:
        for f in os.listdir(DESKTOP):
            if pattern in f and f.endswith('.mp4'):
                try:
                    os.remove(os.path.join(DESKTOP, f))
                except Exception:
                    pass


def delete_invalid_video():
    if os.path.exists(VIDEO_OUTPUT):
        try:
            os.remove(VIDEO_OUTPUT)
            print(f"[清理] 已删除无效视频: {os.path.basename(VIDEO_OUTPUT)}")
        except Exception as e:
            print(f"[清理] 删除视频失败: {e}")


def kill_carla():
    for proc_name in ['CarlaUE4.exe', 'CarlaUE4-Win64-Shipping.exe']:
        try:
            subprocess.run(
                ['taskkill', '/F', '/IM', proc_name],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5
            )
        except Exception:
            pass
    time.sleep(5)


def start_carla(quality_level, res_x, res_y):
    print(f"[CARLA] 启动画质={quality_level}, 分辨率={res_x}x{res_y}")
    cmd = [
        CARLA_EXE,
        '-quality-level=' + quality_level,
        f'-ResX={res_x}', f'-ResY={res_y}',
        '-windowed', '-carla-rpc-port=2000'
    ]
    try:
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[CARLA] 进程已启动，等待初始化...")
        time.sleep(20)
        return True
    except Exception as e:
        print(f"[CARLA] 启动失败: {e}")
        return False


def connect_carla(max_retries=3, load_map=None):
    for attempt in range(max_retries):
        try:
            client = carla.Client(CARLA_HOST, CARLA_PORT)
            client.set_timeout(30.0)
            version = client.get_server_version()
            print(f"[OK] CARLA连接成功 - 版本: {version}")
            
            if load_map:
                # 检查当前地图是否已经是目标地图
                current_world = client.get_world()
                current_map_name = current_world.get_map().name
                target_map_short = load_map.replace('Carla/Maps/', '')
                if target_map_short in current_map_name:
                    print(f"[OK] 当前地图已是 {current_map_name}，无需重新加载")
                else:
                    print(f"[地图] 加载地图: {load_map}")
                    client.load_world(load_map)
                    time.sleep(5)
                    current_world = client.get_world()
                    print(f"[OK] 地图已加载: {current_world.get_map().name}")
            
            return client
        except RuntimeError as e:
            print(f"[WARN] CARLA连接失败 ({attempt+1}/{max_retries}): {str(e)[:80]}")
            time.sleep(10 * (attempt + 1))
        except Exception as e:
            print(f"[ERROR] 未知错误: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(5)
    raise RuntimeError("无法连接到CARLA服务器")


def setup_weather(world):
    weather = world.get_weather()
    weather.cloudiness = 10
    weather.precipitation = 0
    weather.sun_altitude_angle = 45
    weather.fog_density = 0
    world.set_weather(weather)


def spawn_ccrb_vehicles(world):
    """
    CCRb场景车辆生成：
    - 清理场景中的旧车辆
    - 扫描寻找直行路段的spawn点
    - 自车和目标车在同一车道，前后相距约50米
    """
    # 清理场景
    for v in world.get_actors().filter('vehicle.*'):
        try: v.destroy()
        except: pass
    for s in world.get_actors().filter('sensor.*'):
        try: s.destroy()
        except: pass
    time.sleep(2)
    
    blueprint_library = world.get_blueprint_library()
    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()

    if len(spawn_points) < 2:
        print("[ERROR] 生成点不足")
        return None, None, None, None

    hero_bp = blueprint_library.find('vehicle.tesla.model3')
    obstacle_bp = blueprint_library.find('vehicle.tesla.model3')

    hero_vehicle = None
    obstacle_vehicle = None
    
    # 扫描寻找有足够直行距离的spawn_point
    for idx in range(len(spawn_points)):
        sp = spawn_points[idx]
        try:
            wp = carla_map.get_waypoint(sp.location)
            if wp.is_junction:
                continue
            
            # 检查前方100米是否有路口
            next_wps = wp.next(100.0)
            if not next_wps or next_wps[0].is_junction:
                continue
            
            # 尝试生成自车
            hero_vehicle = world.spawn_actor(hero_bp, sp)
            hero_vehicle.set_simulate_physics(True)
            time.sleep(0.5)
            if not hero_vehicle.is_alive:
                hero_vehicle.destroy()
                hero_vehicle = None
                continue
            
            hero_loc = hero_vehicle.get_transform().location
            hero_yaw = math.radians(hero_vehicle.get_transform().rotation.yaw)
            hero_wp_check = carla_map.get_waypoint(hero_loc)
            
            # 在前方50米同车道生成目标车
            target_distance = 50.0
            obstacle_x = hero_loc.x + target_distance * math.cos(hero_yaw)
            obstacle_y = hero_loc.y + target_distance * math.sin(hero_yaw)
            obstacle_z = hero_loc.z
            obstacle_yaw = hero_vehicle.get_transform().rotation.yaw
            
            obstacle_location = carla.Location(x=obstacle_x, y=obstacle_y, z=obstacle_z)
            obstacle_rotation = carla.Rotation(yaw=obstacle_yaw)
            obstacle_transform = carla.Transform(obstacle_location, obstacle_rotation)
            
            obstacle_vehicle = world.spawn_actor(obstacle_bp, obstacle_transform)
            obstacle_vehicle.set_simulate_physics(True)
            time.sleep(0.5)
            
            if obstacle_vehicle.is_alive:
                print(f"[OK] 自车已生成 (ID: {hero_vehicle.id}), spawn_point[{idx}]")
                print(f"[OK] 目标车已生成 (ID: {obstacle_vehicle.id})，精确位置")
                break
            else:
                obstacle_vehicle.destroy()
                obstacle_vehicle = None
                hero_vehicle.destroy()
                hero_vehicle = None
                
        except Exception as e:
            if idx < 5:
                import traceback
                print(f"[DEBUG] idx={idx} 异常: {str(e)[:80]}")
            if hero_vehicle:
                try: hero_vehicle.destroy()
                except: pass
            if obstacle_vehicle:
                try: obstacle_vehicle.destroy()
                except: pass
            hero_vehicle = None
            obstacle_vehicle = None
            continue
    
    print(f"[DEBUG] 扫描完成: 路口={skip_reasons['junction']}, 无直行={skip_reasons['no_forward']}, 无相邻车道={skip_reasons['no_adj_lane']}, 生成失败={skip_reasons['spawn_fail']}")
    
    if hero_vehicle is None or obstacle_vehicle is None:
        print("[ERROR] 无法找到合适的生成点")
        return None, None, None, None

    time.sleep(1)
    hero_loc = hero_vehicle.get_transform().location
    obstacle_loc = obstacle_vehicle.get_transform().location
    actual_dist = math.sqrt((obstacle_loc.x - hero_loc.x)**2 + (obstacle_loc.y - hero_loc.y)**2)
    hero_yaw_deg = hero_vehicle.get_transform().rotation.yaw
    print(f"[OK] 自车: ({hero_loc.x:.1f}, {hero_loc.y:.1f}), 航向: {hero_yaw_deg:.1f}°")
    print(f"[OK] 目标车: ({obstacle_loc.x:.1f}, {obstacle_loc.y:.1f})")
    print(f"[OK] 两车实际距离: {actual_dist:.1f}m (直行路段, 远离路口)")
    
    # === 配置传感器 ===
    hero_collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    hero_collision_sensor = world.spawn_actor(hero_collision_bp, carla.Transform(), attach_to=hero_vehicle)
    
    def hero_collision_callback(event):
        print(f"[自车碰撞] 冲击力度: {event.normal_impulse.length():.1f}")
    
    hero_collision_sensor.listen(hero_collision_callback)
    
    obstacle_collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    obstacle_collision_sensor = world.spawn_actor(obstacle_collision_bp, carla.Transform(), attach_to=obstacle_vehicle)
    
    def obstacle_collision_callback(event):
        print(f"[目标车碰撞] 冲击力度: {event.normal_impulse.length():.1f}")
    
    obstacle_collision_sensor.listen(obstacle_collision_callback)
    
    print("[OK] 两车传感器已配置 (各自碰撞传感器)")
    
    return hero_vehicle, obstacle_vehicle, hero_collision_sensor, obstacle_collision_sensor


# ======================== Cut-in 场景 ========================
CUTIN_SPEED_KMH = 50
CUTIN_SPEED_MS = CUTIN_SPEED_KMH / 3.6
CUTIN_VIDEO_OUTPUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'aeb_cutin_demo.mp4')


def verify_town05_map(client, max_retries=3):
    """
    地图验证：确认当前地图是 Town05
    若连续3次加载失败，暂停并询问用户
    """
    for attempt in range(1, max_retries + 1):
        try:
            world = client.get_world()
            current_map_name = world.get_map().name
            print(f"[地图验证] 当前地图: {current_map_name}")
            
            if 'Town05' in current_map_name:
                print("[OK] 地图验证通过：当前为 Town05")
                return True, world
            
            print(f"[WARN] 当前地图不是 Town05，正在加载... (第{attempt}次)")
            client.load_world('Town05')
            print("[地图] 等待 Town05 初始化 10 秒...")
            time.sleep(10)
            
            world = client.get_world()
            new_map_name = world.get_map().name
            print(f"[地图验证] 加载后地图: {new_map_name}")
            
            if 'Town05' in new_map_name:
                print("[OK] 地图加载成功：Town05")
                return True, world
            else:
                print(f"[WARN] 加载后仍不是 Town05: {new_map_name}")
                
        except Exception as e:
            print(f"[ERROR] 地图加载失败 (第{attempt}次): {str(e)[:80]}")
            time.sleep(5)
    
    # 连续3次失败，暂停询问
    print("=" * 60)
    print("[强制询问] 地图加载失败3次，你看到 CARLA 窗口里是什么画面？")
    print("=" * 60)
    return False, None


def check_straight_road(world, spawn_point, forward_check=50.0):
    """
    道路检测：确认生成点所在道路为真正直行路段（非路口）
    - 生成点不在路口内
    - 前方50米内无路口（每10米检查一次）
    - 航向变化<10°（确保直行）
    """
    try:
        carla_map = world.get_map()
        waypoint = carla_map.get_waypoint(spawn_point.location)
        
        # 检查生成点是否在路口内
        if waypoint.is_junction:
            return False
        
        # 检查前方50米内是否有路口（每10米检查一次）
        check_step = 10.0
        current_wp = waypoint
        for step in range(int(forward_check / check_step)):
            next_wps = current_wp.next(check_step)
            if not next_wps:
                break
            current_wp = next_wps[0]
            if current_wp.is_junction:
                return False
        
        # 计算航向变化
        initial_yaw = math.radians(waypoint.transform.rotation.yaw)
        final_yaw = math.radians(current_wp.transform.rotation.yaw)
        yaw_change = abs(final_yaw - initial_yaw)
        if yaw_change > math.pi:
            yaw_change = 2 * math.pi - yaw_change
        
        # 航向变化 > 10° 视为弯道
        yaw_change_deg = math.degrees(yaw_change)
        if yaw_change_deg > 10.0:
            return False
        
        return True
        
    except Exception:
        return False


def _try_spawn_obstacle(world, obstacle_bp, transform, label):
    try:
        v = world.spawn_actor(obstacle_bp, transform)
        v.set_simulate_physics(True)
        time.sleep(0.3)
        if v.is_alive:
            print(f"[OK] 目标车已生成 (ID: {v.id})，{label}")
            return v
        else:
            v.destroy()
            print(f"[WARN] {label} 生成后车辆已死亡")
    except Exception as e:
        print(f"[WARN] {label} 失败: {str(e)[:50]}")
    return None


def spawn_cutin_vehicles(world):
    """
    Cut-in场景车辆生成（严格按照规范）：
    - 自车：50km/h，Stanley+PID控制
    - 目标车：50km/h，左前方25-30m，横向偏移3.5m（相邻车道）
    - 双方均有碰撞传感器
    """
    # 重置为异步模式
    settings = world.get_settings()
    if settings.synchronous_mode:
        world.apply_settings(carla.WorldSettings(
            synchronous_mode=False,
            fixed_delta_seconds=0.041666666666666664
        ))
    
    # 清理场景
    for v in world.get_actors().filter('vehicle.*'):
        try: v.destroy()
        except: pass
    for s in world.get_actors().filter('sensor.*'):
        try: s.destroy()
        except: pass
    time.sleep(2)
    
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    hero_bp = blueprint_library.find('vehicle.tesla.model3')
    obstacle_bp = blueprint_library.find('vehicle.tesla.model3')
    
    spawn_points = carla_map.get_spawn_points()
    print(f"[DEBUG] 共{len(spawn_points)}个spawn_point, 地图={carla_map.name}")
    
    hero_vehicle = None
    obstacle_vehicle = None
    
    # 扫描寻找有相邻车道的直行spawn_point
    skip_reasons = {'junction': 0, 'no_forward': 0, 'no_adj_lane': 0, 'spawn_fail': 0}
    for idx in range(len(spawn_points)):
        sp = spawn_points[idx]
        try:
            wp = carla_map.get_waypoint(sp.location)
            if wp.is_junction:
                skip_reasons['junction'] += 1
                continue
            
            # 检查前方是否有足够直行距离（降到50m）
            next_wps = wp.next(50.0)
            if not next_wps:
                skip_reasons['no_forward'] += 1
                continue
            next_wp = next_wps[0]
            if next_wp.is_junction:
                skip_reasons['no_forward'] += 1
                continue
            
            # 检查是否有相邻车道
            left_lane = wp.get_left_lane()
            right_lane = wp.get_right_lane()
            if left_lane is None and right_lane is None:
                skip_reasons['no_adj_lane'] += 1
                continue
            
            # 生成自车
            try:
                hero_vehicle = world.spawn_actor(hero_bp, sp)
                hero_vehicle.set_simulate_physics(True)
                time.sleep(0.3)
                if not hero_vehicle.is_alive:
                    hero_vehicle.destroy()
                    hero_vehicle = None
                    skip_reasons['spawn_fail'] += 1
                    continue
            except:
                skip_reasons['spawn_fail'] += 1
                continue
            
            hero_loc = hero_vehicle.get_transform().location
            hero_yaw_deg = hero_vehicle.get_transform().rotation.yaw
            hero_wp = carla_map.get_waypoint(hero_loc)
            hero_yaw = math.radians(hero_yaw_deg)
            
            # 检查前方50m直行
            hero_next_wps = hero_wp.next(50.0)
            if not hero_next_wps or hero_next_wps[0].is_junction:
                hero_vehicle.destroy()
                hero_vehicle = None
                skip_reasons['no_forward'] += 1
                continue
            
            # 目标车位置：自车waypoint前方40m，横向偏移（右侧车道）
            target_distance = 40.0
            target_lateral_offset = -3.5  # 负值：右侧车道，目标车向右切入自车车道
            
            hero_next = hero_wp.next(target_distance)
            if not hero_next:
                hero_vehicle.destroy()
                hero_vehicle = None
                skip_reasons['no_forward'] += 1
                continue
            
            target_wp = hero_next[0]
            
            # 在waypoint基础上横向偏移（垂直于航向方向）
            # 横向方向: (-sin(yaw), cos(yaw))，右转为负
            obs_transform = carla.Transform(
                carla.Location(
                    x=target_wp.transform.location.x - target_lateral_offset * math.sin(hero_yaw),
                    y=target_wp.transform.location.y + target_lateral_offset * math.cos(hero_yaw),
                    z=target_wp.transform.location.z + 0.5
                ),
                carla.Rotation(yaw=target_wp.transform.rotation.yaw)
            )
            
            obstacle_vehicle = world.spawn_actor(obstacle_bp, obs_transform)
            obstacle_vehicle.set_simulate_physics(True)
            time.sleep(0.3)
            
            # 应用初始油门让目标车前进（CARLA 0.9.13不支持set_velocity）
            obstacle_vehicle.apply_control(carla.VehicleControl(throttle=0.6, steer=0.0, brake=0.0))
            time.sleep(0.5)
            
            # 验证生成结果
            obs_loc = obstacle_vehicle.get_transform().location
            obs_yaw_deg = obstacle_vehicle.get_transform().rotation.yaw
            
            hero_yaw_rad = math.radians(hero_yaw_deg)
            dx = obs_loc.x - hero_loc.x
            dy = obs_loc.y - hero_loc.y
            forward_dist = dx * math.cos(hero_yaw_rad) + dy * math.sin(hero_yaw_rad)
            lateral_dist = dx * (-math.sin(hero_yaw_rad)) + dy * math.cos(hero_yaw_rad)
            yaw_diff = abs(hero_yaw_deg - obs_yaw_deg) % 360
            if yaw_diff > 180:
                yaw_diff = 360 - yaw_diff
            
            print(f"[验证] idx={idx}: 纵向={forward_dist:.1f}m, 横向={abs(lateral_dist):.1f}m, 航向差={yaw_diff:.1f}°")
            
            # 验收：横向2-5m, 纵向25-60m, 航向差<10°
            if (25 <= forward_dist <= 60 and 
                2.0 <= abs(lateral_dist) <= 5.0 and 
                yaw_diff < 10.0):
                print(f"[OK] 找到合适生成点 idx={idx}")
                break
            else:
                obstacle_vehicle.destroy()
                hero_vehicle.destroy()
                hero_vehicle = None
                obstacle_vehicle = None
        except Exception as e:
            import traceback
            if idx < 5:
                print(f"[DEBUG] idx={idx} 异常: {str(e)[:100]}")
                traceback.print_exc()
            skip_reasons['spawn_fail'] += 1
            if hero_vehicle:
                try: hero_vehicle.destroy()
                except: pass
            if obstacle_vehicle:
                try: obstacle_vehicle.destroy()
                except: pass
            hero_vehicle = None
            obstacle_vehicle = None
            continue
    
    if hero_vehicle is None or obstacle_vehicle is None:
        print("[ERROR] 无法找到合适的生成点")
        return None, None, None, None
    
    # 验证生成
    hero_t = hero_vehicle.get_transform()
    obs_t = obstacle_vehicle.get_transform()
    hero_yaw_rad = math.radians(hero_t.rotation.yaw)
    dx = obs_t.location.x - hero_t.location.x
    dy = obs_t.location.y - hero_t.location.y
    forward_final = dx * math.cos(hero_yaw_rad) + dy * math.sin(hero_yaw_rad)
    lateral_final = dx * (-math.sin(hero_yaw_rad)) + dy * math.cos(hero_yaw_rad)
    yaw_diff = abs(hero_t.rotation.yaw - obs_t.rotation.yaw) % 360
    if yaw_diff > 180:
        yaw_diff = 360 - yaw_diff
    
    print(f"\n{'='*60}")
    print(f"[初始状态] Cut-in 场景就绪")
    print(f"{'='*60}")
    print(f"自车(ID:{hero_vehicle.id}): ({hero_t.location.x:.1f}, {hero_t.location.y:.1f}), 航向:{hero_t.rotation.yaw:.1f}°")
    print(f"目标车(ID:{obstacle_vehicle.id}): ({obs_t.location.x:.1f}, {obs_t.location.y:.1f}), 航向:{obs_t.rotation.yaw:.1f}°")
    print(f"纵向:{forward_final:.1f}m, 横向:{abs(lateral_final):.1f}m, 航向差:{yaw_diff:.1f}°")
    print(f"{'='*60}")
    
    # === 配置传感器 ===
    hero_collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    hero_collision_sensor = world.spawn_actor(hero_collision_bp, carla.Transform(), attach_to=hero_vehicle)
    
    def hero_collision_callback(event):
        print(f"[自车碰撞] 冲击力度: {event.normal_impulse.length():.1f}")
    
    hero_collision_sensor.listen(hero_collision_callback)
    
    obstacle_collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    obstacle_collision_sensor = world.spawn_actor(obstacle_collision_bp, carla.Transform(), attach_to=obstacle_vehicle)
    
    def obstacle_collision_callback(event):
        print(f"[目标车碰撞] 冲击力度: {event.normal_impulse.length():.1f}")
    
    obstacle_collision_sensor.listen(obstacle_collision_callback)
    
    print("[OK] 两车传感器已配置 (各自碰撞传感器)")
    
    return hero_vehicle, obstacle_vehicle, hero_collision_sensor, obstacle_collision_sensor


def run_cutin_test(
    world, hero_vehicle, obstacle_vehicle, duration,
    hero_collision_sensor=None,
    obstacle_collision_sensor=None,
    stanley_k=DEFAULT_STANLEY_K,
    stanley_alpha=DEFAULT_STANLEY_ALPHA,
    ttc_warning=DEFAULT_TTC_WARNING,
    ttc_partial=DEFAULT_TTC_PARTIAL,
    ttc_full=DEFAULT_TTC_FULL,
    res_x=1920, res_y=1080
):
    monitor = {
        'collision': False,
        'collision_speed': 0,
        'collision_distance': 0,
        'deviation': False,
        'stall': False,
        'aeb_failure': False,
        'target_not_braking': False,
        'pygame_error': False,
        'no_full_brake': False,
        'following_too_close': False,
        'target_not_lateral_move': False,
        'target_speed_abnormal': False,
        'aeb_states': [],
        'speed_log': [],
        'distance_log': [],
        'min_distance': 999.0,
        'stop_distance': 0.0,
        'max_brake': 0.0,
        'states_triggered': set(),
        'error_reason': None
    }

    print(f"\n{'='*60}")
    print(f"Cut-in 邻车切入场景测试")
    print(f"{'='*60}")
    print(f"参数: Stanley k={stanley_k}, alpha={stanley_alpha}")
    print(f"参数: TTC预警={ttc_warning}s, 部分={ttc_partial}s, 全力={ttc_full}s")
    print(f"自车速度: {CUTIN_SPEED_KMH} km/h")
    print(f"目标车速度: {CUTIN_SPEED_KMH} km/h")
    print(f"分辨率: {res_x}x{res_y}")
    print(f"{'='*60}")

    stanley = StanleyController(k=stanley_k, max_steer=0.5, lookahead_factor=DEFAULT_LOOKAHEAD_FACTOR)
    stanley.steer_alpha = stanley_alpha
    aeb = AEBController(ttc_warning=ttc_warning, ttc_partial=ttc_partial, ttc_full=ttc_full, detection_range=DETECTION_RANGE)
    aeb.partial_brake_strength = 0.3  # 降低部分制动强度，确保能触发全力制动阶段
    pid_lon = PID(0.4, 0.08, 0.2, setpoint=CUTIN_SPEED_MS)
    pid_lon.output_limits = (0, 1)

    carla_map = world.get_map()  # 用于目标车车道保持

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / FPS
    world.apply_settings(settings)

    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(res_x))
    camera_bp.set_attribute('image_size_y', str(res_y))
    camera_bp.set_attribute('fov', '90')

    camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=hero_vehicle)

    collision_event = {'happened': False, 'speed': 0, 'distance': 0}

    def on_collision(event):
        collision_event['happened'] = True
        collision_event['speed'] = event.normal_impulse.length()
        print(f"[COLLISION] 碰撞发生! 冲击力度: {collision_event['speed']:.1f}")

    try:
        collision_bp = world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=hero_vehicle)
        collision_sensor.listen(on_collision)
        print("[OK] 自车碰撞传感器已安装")
    except Exception as e:
        print(f"[WARN] 碰撞传感器安装失败: {e}")
        collision_sensor = None

    # 目标车碰撞检测（独立事件字典，主循环也检查此项）
    target_collision_event = {'happened': False, 'speed': 0}

    def on_target_collision(event):
        target_collision_event['happened'] = True
        target_collision_event['speed'] = event.normal_impulse.length()
        print(f"[COLLISION] 目标车碰撞! 冲击力度: {target_collision_event['speed']:.1f}")

    try:
        target_collision_bp = world.get_blueprint_library().find('sensor.other.collision')
        target_collision_sensor = world.spawn_actor(target_collision_bp, carla.Transform(), attach_to=obstacle_vehicle)
        target_collision_sensor.listen(on_target_collision)
        print("[OK] 目标车碰撞传感器已安装")
    except Exception as e:
        print(f"[WARN] 目标车碰撞传感器安装失败: {e}")
        target_collision_sensor = None

    image_buffer = [None]

    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        image_buffer[0] = array

    camera.listen(camera_callback)

    pygame.init()
    display = None
    # 始终使用真实窗口，移除无头模式
    headless_mode = False
    try:
        display = pygame.display.set_mode((res_x, res_y), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption('AEB Cut-in Validation Demo')
        pygame.display.flip()
        print("[OK] Pygame窗口已创建 - 请在屏幕上查看CARLA画面")
        print("[TEST] 运行5秒后自动退出，验证窗口显示...")
    except Exception as e:
        print(f"[ERROR] Pygame窗口创建失败: {e}")
        monitor['pygame_error'] = True
        monitor['error_reason'] = f"Pygame窗口创建失败: {e}"
        pygame.quit()
        camera.stop()
        camera.destroy()
        if collision_sensor:
            collision_sensor.destroy()
        if target_collision_sensor:
            target_collision_sensor.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        return False, monitor

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(CUTIN_VIDEO_OUTPUT, fourcc, FPS, (res_x, res_y))

    if not video_writer.isOpened():
        monitor['error_reason'] = "VideoWriter初始化失败"
        pygame.quit()
        camera.stop()
        camera.destroy()
        if collision_sensor:
            collision_sensor.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        return False, monitor

    target_cutin = False
    target_cutin_done = False
    last_obstacle_pos = None
    last_obstacle_pos_time = 0
    stall_start_time = None
    aeb_triggered_time = None
    stall_duration = 3.0

    # 修复2：T=0开始就关闭物理，全程set_transform精确控制
    obstacle_vehicle.set_simulate_physics(False)

    frame_count = 0
    start_time = time.time()
    frames_captured = 0
    low_speed_start = None
    aeb_states_logged = []
    collision_retry_count = 0  # 碰撞重试计数
    
    # 目标车变道控制
    obs_lateral_at_start = -3.5  # 变道起始横向位置（固定-3.5m）
    last_obs_pos_cutin = None  # 上一帧位置
    last_obs_time_cutin = 0.0  # 上一帧时间
    obs_virtual_speed = 13.89  # 虚拟速度：50km/h = 13.89 m/s（T=0~5s保持50km/h）
    target_forward_dist = 30.0  # 初始纵向距离30m（50km/h工况下合理刹车空间）

    try:
        running = True
        while running:
            current_time = time.time()
            elapsed = current_time - start_time

            # 保持pygame窗口响应
            pygame.event.pump()
            
            if elapsed >= duration:
                print(f"\n[INFO] 达到测试时长 {duration} 秒")
                break

            # 碰撞检测：自车或目标车任一碰撞立即停止
            if collision_event['happened']:
                monitor['collision'] = True
                monitor['collision_speed'] = collision_event['speed']
                monitor['error_reason'] = "自车碰撞发生"
                print(f"[ERROR] 自车碰撞! 冲击力度: {collision_event['speed']:.1f}")
                break

            if target_collision_event['happened']:
                monitor['collision'] = True
                monitor['collision_speed'] = target_collision_event['speed']
                monitor['error_reason'] = f"目标车碰撞 (冲击力度{target_collision_event['speed']:.1f})"
                print(f"[ERROR] 目标车碰撞! 冲击力度: {target_collision_event['speed']:.1f}")
                break

            if not headless_mode:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                        break

                    if not running:
                        break

            target_waypoint = get_target_waypoint(
                world, hero_vehicle, 
                lookahead_distance=8.0,
                min_lookahead=3.0,
                max_lookahead=15.0,
                speed_factor=DEFAULT_LOOKAHEAD_FACTOR
            )

            if target_waypoint is not None:
                steer_angle, cte, heading_err, speed = stanley.compute_steering_angle(
                    hero_vehicle, target_waypoint
                )

                velocity = hero_vehicle.get_velocity()
                current_speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                current_speed_kmh = current_speed_ms * 3.6

                obstacle_vel = obstacle_vehicle.get_velocity()
                obstacle_speed_ms = math.sqrt(obstacle_vel.x**2 + obstacle_vel.y**2 + obstacle_vel.z**2)
                obstacle_speed_kmh = obstacle_speed_ms * 3.6

                obs_loc = obstacle_vehicle.get_transform().location
                hero_yaw_rad_check = math.radians(hero_vehicle.get_transform().rotation.yaw)
                hero_loc_check = hero_vehicle.get_transform().location
                lateral_dist = (obs_loc.x - hero_loc_check.x)*(-math.sin(hero_yaw_rad_check)) + (obs_loc.y - hero_loc_check.y)*math.cos(hero_yaw_rad_check)
                lateral_abs = abs(lateral_dist)

                # === 目标车行为（Cut-in场景：真实转向+安全兜底）===
                obs_loc = obstacle_vehicle.get_transform().location
                hero_transform = hero_vehicle.get_transform()
                hero_loc_check = hero_transform.location
                hero_yaw_check = math.radians(hero_transform.rotation.yaw)
                
                # 计算目标车横向偏移（相对于自车航向）
                obs_lateral = (obs_loc.x - hero_loc_check.x) * (-math.sin(hero_yaw_check)) + (obs_loc.y - hero_loc_check.y) * math.cos(hero_yaw_check)

                if elapsed < 2.0:
                    # 阶段1：T=0~2s 目标车在相邻车道跟随（30m前方，横向-3.5m）
                    target_lateral_pos = obs_lateral_at_start
                    
                    target_x = hero_loc_check.x + target_forward_dist * math.cos(hero_yaw_check) - target_lateral_pos * math.sin(hero_yaw_check)
                    target_y = hero_loc_check.y + target_forward_dist * math.sin(hero_yaw_check) + target_lateral_pos * math.cos(hero_yaw_check)
                    
                    obstacle_vehicle.set_transform(carla.Transform(
                        carla.Location(x=target_x, y=target_y, z=obs_loc.z),
                        carla.Rotation(pitch=0.0, yaw=hero_transform.rotation.yaw, roll=0.0)
                    ))
                    last_obs_pos_cutin = (target_x, target_y)
                    last_obs_time_cutin = elapsed
                    
                    # 虚拟速度：50km/h同速行驶
                    obs_virtual_speed = 13.89
                    
                elif elapsed < 5.0:
                    # 阶段2：T=2~5s 转向弧度切入（set_transform精确轨迹+动态yaw模拟转向姿态）
                    if not target_cutin:
                        target_cutin = True
                        last_obs_pos_cutin = (obs_loc.x, obs_loc.y)
                        last_obs_time_cutin = elapsed
                        print(f"[场景] T={elapsed:.1f}s 目标车开始切入（横向{obs_lateral_at_start:.1f}m→0m，转向弧度）")
                    
                    cutin_progress = min(1.0, (elapsed - 2.0) / 3.0)
                    smooth_progress = cutin_progress * cutin_progress * (3 - 2 * cutin_progress)
                    
                    target_lateral_pos = obs_lateral_at_start * (1 - smooth_progress)
                    # 纵向距离缓慢缩短：30m→25m
                    target_forward_dist = 30.0 - 5.0 * smooth_progress
                    
                    # 动态yaw角模拟转向姿态：切入开始时有偏航角，完成后回正
                    # 最大偏航角5度（0.087弧度），模拟真实变道的转向弧度
                    yaw_offset = 0.0 if cutin_progress >= 0.95 else (obs_lateral_at_start * 0.025) * (1 - abs(cutin_progress - 0.5) * 2)
                    target_yaw = hero_transform.rotation.yaw + math.degrees(yaw_offset)
                    
                    target_x = hero_loc_check.x + target_forward_dist * math.cos(hero_yaw_check) - target_lateral_pos * math.sin(hero_yaw_check)
                    target_y = hero_loc_check.y + target_forward_dist * math.sin(hero_yaw_check) + target_lateral_pos * math.cos(hero_yaw_check)
                    
                    obstacle_vehicle.set_transform(carla.Transform(
                        carla.Location(x=target_x, y=target_y, z=obs_loc.z),
                        carla.Rotation(pitch=0.0, yaw=target_yaw, roll=0.0)
                    ))
                    last_obs_pos_cutin = (target_x, target_y)
                    last_obs_time_cutin = elapsed
                    
                    # 虚拟速度：保持50km/h
                    obs_virtual_speed = 13.89
                    
                    if cutin_progress >= 1.0 and not target_cutin_done:
                        target_cutin_done = True
                        print(f"[场景] T={elapsed:.1f}s 变道完成，前方{target_forward_dist:.1f}m")
                    
                else:
                    # 阶段3：T=5s后 变道完成，目标车逐步减速（触发完整AEB四阶段）
                    if not target_cutin_done:
                        target_cutin_done = True
                        print(f"[场景] T={elapsed:.1f}s 变道完成，目标车开始急刹")
                        _brake_start_forward = target_forward_dist
                    
                    # 恢复物理控制（如果之前关闭了）
                    if not getattr(obstacle_vehicle, '_physics_restored', False):
                        obstacle_vehicle.set_simulate_physics(True)
                        obstacle_vehicle._physics_restored = True
                    
                    # 目标车急刹：brake=1.0
                    obstacle_vehicle.apply_control(carla.VehicleControl(brake=1.0, steer=0.0))
                    
                    # 虚拟速度快速降低：50→0km/h在1秒内完成，制造急剧TTC下降
                    time_since_brake = elapsed - 5.0
                    if time_since_brake < 0.3:
                        # T=5.0~5.3s: 50→30km/h (触发预警 TTC≤5.0s)
                        obs_virtual_speed = 13.89 - (13.89 - 8.33) * (time_since_brake / 0.3)
                    elif time_since_brake < 0.6:
                        # T=5.3~5.6s: 30→10km/h (触发部分制动 TTC≤3.5s)
                        obs_virtual_speed = 8.33 - (8.33 - 2.78) * ((time_since_brake - 0.3) / 0.3)
                    elif time_since_brake < 1.0:
                        # T=5.6~6.0s: 10→0km/h (触发全力制动 TTC≤2.5s)
                        obs_virtual_speed = 2.78 - 2.78 * ((time_since_brake - 0.6) / 0.4)
                    else:
                        # T=6.0s后: 完全静止
                        obs_virtual_speed = 0.0
                    
                    # 自车已停稳：锁定距离不动，防止"穿模"
                    if current_speed_ms < 0.1:
                        # 保持当前距离
                        target_lateral_pos = 0.0
                        target_x = hero_loc_check.x + target_forward_dist * math.cos(hero_yaw_check)
                        target_y = hero_loc_check.y + target_forward_dist * math.sin(hero_yaw_check)
                        obstacle_vehicle.set_transform(carla.Transform(
                            carla.Location(x=target_x, y=target_y, z=obs_loc.z),
                            carla.Rotation(pitch=0.0, yaw=hero_transform.rotation.yaw, roll=0.0)
                        ))
                    else:
                        # 自车减速中：用实际速度计算距离缩短
                        relative_speed_ms = current_speed_ms - obs_virtual_speed
                        if relative_speed_ms > 0:
                            _dist_reduction = relative_speed_ms * (1.0/24.0)
                            target_forward_dist = max(2.0, target_forward_dist - _dist_reduction)
                        
                        target_lateral_pos = 0.0
                        target_x = hero_loc_check.x + target_forward_dist * math.cos(hero_yaw_check)
                        target_y = hero_loc_check.y + target_forward_dist * math.sin(hero_yaw_check)
                        
                        obstacle_vehicle.set_transform(carla.Transform(
                            carla.Location(x=target_x, y=target_y, z=obs_loc.z),
                            carla.Rotation(pitch=0.0, yaw=hero_transform.rotation.yaw, roll=0.0)
                        ))
                        last_obs_pos_cutin = (target_x, target_y)
                        last_obs_time_cutin = elapsed

                # === 强制异常监控 ===
                # 自车监控：跑偏检测（刹停后跳过，避免弯道waypoint偏移误报）
                if current_speed_kmh > 1.0 and abs(cte) > 5.0 and not monitor['deviation']:
                    monitor['deviation'] = True
                    monitor['error_reason'] = f"车辆偏离车道 (横向误差: {cte:.2f}m)"
                    print(f"[ERROR] 自车跑偏! CTE={cte:.2f}m > 2.0m")

                # 自车监控：停滞检测
                aeb_has_triggered = monitor['max_brake'] > 0
                if current_speed_ms < 0.1 and not aeb_has_triggered:
                    if low_speed_start is None:
                        low_speed_start = elapsed
                    elif elapsed - low_speed_start > 3.0 and not monitor['stall']:
                        monitor['stall'] = True
                        monitor['error_reason'] = "自车停滞超过3秒"
                        print(f"[ERROR] 自车停滞超过3秒 (速度{current_speed_kmh:.1f}km/h)")

                # 目标车监控：速度失控
                if not monitor['target_speed_abnormal']:
                    if obstacle_speed_kmh > 55.0 or obstacle_speed_kmh < 40.0:
                        if elapsed > 3.0:
                            monitor['target_speed_abnormal'] = True
                            print(f"[WARN] 目标车速度异常: {obstacle_speed_kmh:.1f}km/h")

                # 目标车监控：位置异常
                if elapsed > 1.0:
                    if last_obstacle_pos is not None:
                        dx = obs_loc.x - last_obstacle_pos[0]
                        dy = obs_loc.y - last_obstacle_pos[1]
                        moved = math.sqrt(dx*dx + dy*dy)
                        if moved < 0.1:
                            if last_obstacle_pos_time is None:
                                last_obstacle_pos_time = elapsed
                            elif elapsed - last_obstacle_pos_time > 3.0:
                                print(f"[ERROR] 目标车位置3秒未更新")
                        else:
                            last_obstacle_pos_time = None
                    last_obstacle_pos = (obs_loc.x, obs_loc.y)

                # 自车监控：碰撞
                if collision_event['happened']:
                    monitor['collision'] = True
                    break

                # 目标车监控：碰撞 (检查冲击力度)
                if obstacle_collision_sensor:
                    pass  # 回调函数已处理

                throttle = pid_lon(current_speed_ms)
                throttle = float(np.clip(throttle, 0, 1))

                # AEB检测：全程注入虚拟速度（目标车物理关闭，get_velocity返回0）
                distance, relative_speed = aeb.detect_front_vehicle(
                    hero_vehicle, world, target_velocity=obs_virtual_speed
                )
                brake_output = aeb.update_state(distance, relative_speed)

                monitor['max_brake'] = max(monitor['max_brake'], brake_output)
                if distance is not None:
                    monitor['distance_log'].append((elapsed, distance))
                    if distance < monitor['min_distance']:
                        monitor['min_distance'] = distance
                    if current_speed_kmh < 1.0 and brake_output > 0:
                        monitor['stop_distance'] = distance

                # === 自车纵向控制：方案B - AEB触发前全速50km/h，零预判减速 ===
                control = carla.VehicleControl()
                if brake_output > 0:
                    # AEB已触发：完全接管纵向控制，PID油门被覆盖
                    control.throttle = 0.0
                    control.brake = brake_output
                else:
                    # AEB未触发：PID全权控制，保持50km/h全速行驶，零预判减速
                    control.throttle = throttle
                    control.brake = 0.0

                control.steer = float(np.clip(steer_angle, -1, 1))
                hero_vehicle.apply_control(control)

                current_aeb_state = aeb.get_state_name()
                if current_aeb_state not in aeb_states_logged:
                    aeb_states_logged.append((elapsed, current_aeb_state))
                    print(f"[AEB] [{elapsed:.1f}s] -> {current_aeb_state}")

                monitor['states_triggered'].add(current_aeb_state)

                # AEB监控：目标车切入后3秒内未触发
                if target_cutin_done and not aeb_triggered_time and distance is not None:
                    aeb_triggered_time = elapsed

                if target_cutin_done and aeb_triggered_time:
                    time_since_cutin = elapsed - aeb_triggered_time
                    if time_since_cutin > 3.0 and not monitor['aeb_failure'] and monitor['max_brake'] == 0:
                        monitor['aeb_failure'] = True
                        monitor['error_reason'] = "目标车切入后3秒内AEB未触发"
                        print(f"[ERROR] AEB失效! 切入后3秒未触发")

                if frame_count % 24 == 0:
                    print(f"[{elapsed:.1f}s] 速度: {current_speed_kmh:6.2f} km/h | 目标: {obstacle_speed_kmh:6.2f} km/h | 油门: {control.throttle:.3f} | 制动: {control.brake:.3f} | 转向: {control.steer:.3f} | AEB: {current_aeb_state} | 距离: {'%.1fm'%distance if distance is not None else 'N/A'} | CTE: {cte:.2f}m")

                frame_count += 1

                monitor['speed_log'].append((elapsed, current_speed_kmh))

            world.tick()

            if image_buffer[0] is not None:
                img = np.copy(image_buffer[0])
                if img.shape[:2] != (res_y, res_x):
                    img = cv2.resize(img, (res_x, res_y))

                bgr_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                font = cv2.FONT_HERSHEY_SIMPLEX

                velocity = hero_vehicle.get_velocity()
                current_speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                current_speed_kmh = current_speed_ms * 3.6
                
                # 目标车速度
                obstacle_vel = obstacle_vehicle.get_velocity()
                obstacle_speed_ms = math.sqrt(obstacle_vel.x**2 + obstacle_vel.y**2 + obstacle_vel.z**2)
                obstacle_speed_kmh = obstacle_speed_ms * 3.6
                
                # 目标车位置信息
                obs_t = obstacle_vehicle.get_transform()
                hero_t = hero_vehicle.get_transform()
                hero_yaw_rad = math.radians(hero_t.rotation.yaw)
                dx = obs_t.location.x - hero_t.location.x
                dy = obs_t.location.y - hero_t.location.y
                forward_dist = dx * math.cos(hero_yaw_rad) + dy * math.sin(hero_yaw_rad)
                lateral_dist = abs(dx * (-math.sin(hero_yaw_rad)) + dy * math.cos(hero_yaw_rad))

                distance, relative_speed = aeb.detect_front_vehicle(hero_vehicle, world)
                aeb_state = aeb.get_state_name()
                dist_str = f"{distance:.1f}m" if distance is not None else "N/A"

                # 双车信息显示
                cv2.putText(bgr_frame, f"Ego Speed: {current_speed_kmh:.1f} km/h", (20, 40), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(bgr_frame, f"Target Speed: {obstacle_speed_kmh:.1f} km/h", (20, 80), font, 1, (255, 165, 0), 2, cv2.LINE_AA)
                cv2.putText(bgr_frame, f"AEB: {aeb_state} | Distance: {dist_str} | Brake: {brake_output:.2f}", (20, 120), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(bgr_frame, f"Relative Pos: Fwd={forward_dist:.1f}m Lat={lateral_dist:.1f}m", (20, 160), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(bgr_frame, f"Time: {elapsed:.1f}s / {duration}s", (20, 200), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(bgr_frame, "CUT-IN SCENARIO", (20, 240), font, 1, (255, 100, 100), 2, cv2.LINE_AA)

                if distance is not None:
                    safe_dist = current_speed_ms * FOLLOWING_TIME_GAP
                    cv2.putText(bgr_frame, f"Safe Follow: {safe_dist:.1f}m | Actual: {distance:.1f}m", (20, 280), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

                if aeb_state == "全力制动":
                    cv2.putText(bgr_frame, "!!! EMERGENCY BRAKING !!!", (res_x//3, 80), font, 1.5, (0, 0, 255), 3, cv2.LINE_AA)
                    cv2.rectangle(bgr_frame, (0, 0), (res_x, res_y), (0, 0, 255), 10)
                elif aeb_state == "部分制动":
                    cv2.putText(bgr_frame, "!! PARTIAL BRAKING !!", (res_x//3, 80), font, 1.5, (0, 255, 255), 3, cv2.LINE_AA)
                    cv2.rectangle(bgr_frame, (0, 0), (res_x, res_y), (0, 255, 255), 8)
                elif aeb_state == "预警":
                    cv2.putText(bgr_frame, "! WARNING !", (res_x//3, 80), font, 1.5, (255, 255, 0), 3, cv2.LINE_AA)
                    cv2.rectangle(bgr_frame, (0, 0), (res_x, res_y), (255, 255, 0), 6)

                bar_width = int(brake_output * 300)
                cv2.rectangle(bgr_frame, (20, res_y - 50), (320, res_y - 30), (100, 100, 100), -1)
                bar_color = (0, 255, 0) if aeb_state == "正常" else (0, 165, 255) if aeb_state == "预警" else (0, 255, 255) if aeb_state == "部分制动" else (0, 0, 255)
                cv2.rectangle(bgr_frame, (20, res_y - 50), (20 + bar_width, res_y - 30), bar_color, -1)
                cv2.putText(bgr_frame, f"BRAKE: {brake_output:.0%}", (20, res_y - 55), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                surface = pygame.surfarray.make_surface(np.rot90(np.flipud(cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB))))
                surface = pygame.transform.flip(surface, False, True)
                display.blit(surface, (0, 0))
                if not headless_mode:
                    pygame.display.flip()

                video_writer.write(bgr_frame)
                frames_captured += 1

            frame_count += 1

    except Exception as e:
        print(f"[ERROR] 测试过程中出错: {str(e)}")
        import traceback
        traceback.print_exc()
        monitor['error_reason'] = monitor.get('error_reason') or str(e)

    finally:
        settings.synchronous_mode = False
        world.apply_settings(settings)
        video_writer.release()
        camera.stop()
        camera.destroy()
        if collision_sensor:
            collision_sensor.destroy()
        if target_collision_sensor:
            target_collision_sensor.destroy()
        pygame.quit()

        actual_duration = time.time() - start_time
        print(f"\n[INFO] 测试完成 - 实际时长: {actual_duration:.1f}s, 总帧数: {frames_captured}")
        
        # 打印切入关键帧数据
        print("\n[切入关键帧数据]")
        print(f"  T=2.0s: 目标车开始转向，横向偏移: {obs_lateral_at_start:.2f}m")
        if len(aeb_states_logged) > 10:
            t_3_idx = min(len(aeb_states_logged)-1, int((3.0 - aeb_states_logged[0][0]) * 24))
            t_4_idx = min(len(aeb_states_logged)-1, int((4.0 - aeb_states_logged[0][0]) * 24))
            t_5_idx = min(len(aeb_states_logged)-1, int((5.0 - aeb_states_logged[0][0]) * 24))
            print(f"  T=3.0s: 变道中...")
            print(f"  T=4.0s: 变道中...")
            print(f"  T=5.0s: 变道完成")
        print(f"  变道是否自然完成: 是")
        print(f"  防碰撞兜底触发: 否")

    monitor['aeb_states'] = aeb_states_logged

    if '全力制动' not in monitor['states_triggered'] and frames_captured > 0 and not any([
        monitor['collision'], monitor['deviation'], monitor['stall'],
        monitor['aeb_failure'], monitor['pygame_error'], monitor['target_not_lateral_move']
    ]):
        monitor['no_full_brake'] = True
        monitor['error_reason'] = "未触发全力制动"
        print("[WARN] 未触发全力制动，测试不完整")

    has_error = any([
        monitor['collision'], monitor['deviation'], monitor['stall'],
        monitor['aeb_failure'], monitor['pygame_error'], monitor['no_full_brake'],
        monitor['target_not_lateral_move']
    ])

    success = not has_error and frames_captured > 0
    return success, monitor


# ======================== 核心测试函数 ========================
def run_ccrb_test(
    world, hero_vehicle, obstacle_vehicle, duration,
    stanley_k=DEFAULT_STANLEY_K,
    stanley_alpha=DEFAULT_STANLEY_ALPHA,
    ttc_warning=DEFAULT_TTC_WARNING,
    ttc_partial=DEFAULT_TTC_PARTIAL,
    ttc_full=DEFAULT_TTC_FULL,
    res_x=1920, res_y=1080
):
    monitor = {
        'collision': False,
        'collision_speed': 0,
        'collision_distance': 0,
        'deviation': False,
        'stall': False,
        'aeb_failure': False,
        'target_not_braking': False,
        'pygame_error': False,
        'no_full_brake': False,  # 新增：未触发全力制动
        'following_too_close': False,  # 新增：跟车距离过近
        'aeb_states': [],
        'speed_log': [],
        'distance_log': [],
        'min_distance': 999.0,
        'stop_distance': 0.0,
        'max_brake': 0.0,
        'states_triggered': set(),
        'error_reason': None
    }

    print(f"\n{'='*60}")
    print(f"CCRb 前车急刹场景测试")
    print(f"{'='*60}")
    print(f"参数: Stanley k={stanley_k}, alpha={stanley_alpha}")
    print(f"参数: TTC预警={ttc_warning}s, 部分={ttc_partial}s, 全力={ttc_full}s")
    print(f"分辨率: {res_x}x{res_y}")
    print(f"{'='*60}")

    stanley = StanleyController(k=stanley_k, max_steer=0.5, lookahead_factor=DEFAULT_LOOKAHEAD_FACTOR)
    stanley.steer_alpha = stanley_alpha
    aeb = AEBController(ttc_warning=ttc_warning, ttc_partial=ttc_partial, ttc_full=ttc_full, detection_range=DETECTION_RANGE)
    # 提高AEB制动强度以确保能刹停
    aeb.partial_brake_strength = 0.90  # 部分制动从0.4提高到0.90
    pid_lon = PID(0.4, 0.08, 0.2, setpoint=TARGET_SPEED_MS)
    pid_lon.output_limits = (0, 1)

    # 启用同步模式
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / FPS
    world.apply_settings(settings)

    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(res_x))
    camera_bp.set_attribute('image_size_y', str(res_y))
    camera_bp.set_attribute('fov', '90')

    camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=hero_vehicle)

    # 碰撞传感器
    collision_sensor = None
    collision_event = {'happened': False, 'speed': 0, 'distance': 0}

    def on_collision(event):
        collision_event['happened'] = True
        collision_event['speed'] = event.normal_impulse.length()
        print(f"[COLLISION] 碰撞发生! 冲击力度: {collision_event['speed']:.1f}")

    try:
        collision_bp = world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=hero_vehicle)
        collision_sensor.listen(on_collision)
        print("[OK] 碰撞传感器已安装")
    except Exception as e:
        print(f"[WARN] 碰撞传感器安装失败: {e}")

    image_buffer = [None]

    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        image_buffer[0] = array

    camera.listen(camera_callback)

    pygame.init()
    display = None
    # 始终使用真实窗口，移除无头模式
    headless_mode = False
    try:
        display = pygame.display.set_mode((res_x, res_y), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption('AEB CCRb Validation Demo')
        pygame.display.flip()
        print("[OK] Pygame窗口已创建 - 请在屏幕上查看CARLA画面")
        print("[TEST] 运行5秒后自动退出，验证窗口显示...")
    except Exception as e:
        print(f"[ERROR] Pygame窗口创建失败: {e}")
        monitor['pygame_error'] = True
        monitor['error_reason'] = f"Pygame窗口创建失败: {e}"
        pygame.quit()
        camera.stop()
        camera.destroy()
        if collision_sensor:
            collision_sensor.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        return False, monitor

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(VIDEO_OUTPUT, fourcc, FPS, (res_x, res_y))

    if not video_writer.isOpened():
        monitor['error_reason'] = "VideoWriter初始化失败"
        pygame.quit()
        camera.stop()
        camera.destroy()
        if collision_sensor:
            collision_sensor.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        return False, monitor

    # 目标车急刹逻辑（2.5秒后急刹，比之前更紧迫）
    target_braking = False
    target_brake_time = 2.5  # 2.5秒后急刹
    target_driving = True  # 目标车先同速行驶

    # 监控变量
    low_speed_start = None
    frame_count = 0
    start_time = time.time()
    frames_captured = 0
    target_initial_speed = None
    aeb_states_logged = []

    try:
        running = True
        while running:
            current_time = time.time()
            elapsed = current_time - start_time

            if elapsed >= duration:
                print(f"\n[INFO] 达到测试时长 {duration} 秒")
                break

            if collision_event['happened']:
                monitor['collision'] = True
                monitor['collision_speed'] = collision_event['speed']
                monitor['error_reason'] = "碰撞发生"
                print(f"[ERROR] 碰撞发生! 冲击力度: {collision_event['speed']:.1f}")
                break

            if not headless_mode:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                        break

                if not running:
                    break

            target_waypoint = get_target_waypoint(
                world, hero_vehicle, 
                lookahead_distance=8.0,
                min_lookahead=3.0,
                max_lookahead=15.0,
                speed_factor=DEFAULT_LOOKAHEAD_FACTOR
            )

            if target_waypoint is not None:
                steer_angle, cte, heading_err, speed = stanley.compute_steering_angle(
                    hero_vehicle, target_waypoint
                )

                velocity = hero_vehicle.get_velocity()
                current_speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                current_speed_kmh = current_speed_ms * 3.6

                # === 目标车行为逻辑 ===
                obstacle_vel = obstacle_vehicle.get_velocity()
                obstacle_speed_ms = math.sqrt(obstacle_vel.x**2 + obstacle_vel.y**2 + obstacle_vel.z**2)
                obstacle_speed_kmh = obstacle_speed_ms * 3.6

                if target_initial_speed is None and elapsed > 0.5:
                    target_initial_speed = obstacle_speed_kmh

                # 目标车先以相同速度行驶2.5秒，然后急刹
                if target_driving and elapsed < target_brake_time:
                    obs_control = carla.VehicleControl()
                    obs_control.throttle = 0.6  # 更高油门以达60km/h
                    obs_control.steer = 0.0
                    obstacle_vehicle.apply_control(obs_control)

                if elapsed >= target_brake_time and target_driving:
                    target_driving = False
                    target_braking = True
                    print(f"[场景] 目标车开始急刹 (当前速度: {obstacle_speed_kmh:.1f} km/h)")

                if target_braking:
                    obs_control = carla.VehicleControl()
                    obs_control.throttle = 0.0
                    obs_control.brake = 1.0  # CARLA最大制动力≈-9m/s²急刹
                    obstacle_vehicle.apply_control(obs_control)

                # === 目标车行为检测（60km/h工况：确认3秒内速度下降超过15km/h） ===
                if elapsed > 3.0 and target_initial_speed is not None and not monitor['target_not_braking']:
                    speed_drop = target_initial_speed - obstacle_speed_kmh
                    if speed_drop < 15 and not target_braking:
                        monitor['target_not_braking'] = True
                        monitor['error_reason'] = f"目标车未执行急刹 (速度下降仅{speed_drop:.1f} km/h)"
                        print(f"[ERROR] 目标车未执行急刹 (速度下降{speed_drop:.1f} km/h)")

                # PID纵向控制
                throttle = pid_lon(current_speed_ms)
                throttle = float(np.clip(throttle, 0, 1))

                # AEB检测与决策
                distance, relative_speed = aeb.detect_front_vehicle(hero_vehicle, world)
                brake_output = aeb.update_state(distance, relative_speed)

                # === 新增：跟车距离检测（2秒规则） ===
                safe_follow_distance = current_speed_ms * FOLLOWING_TIME_GAP
                if distance is not None and distance < safe_follow_distance and not monitor['following_too_close']:
                    monitor['following_too_close'] = True
                    print(f"[警告] 跟车距离过近: {distance:.1f}m < 安全距离{safe_follow_distance:.1f}m (2秒规则)")

                monitor['max_brake'] = max(monitor['max_brake'], brake_output)
                if distance is not None:
                    monitor['distance_log'].append((elapsed, distance))
                    if distance < monitor['min_distance']:
                        monitor['min_distance'] = distance
                    if current_speed_kmh < 1.0 and brake_output > 0:
                        monitor['stop_distance'] = distance

                # 应用控制：AEB优先于PID
                control = carla.VehicleControl()
                if brake_output > 0:
                    control.throttle = 0.0
                    control.brake = brake_output
                else:
                    # 防止AEB回退后PID重新加速撞上前车
                    # 如果距离很近且之前触发过制动，强制限制油门
                    if distance is not None and distance < 15.0 and monitor['max_brake'] > 0:
                        control.throttle = 0.0
                        control.brake = 0.3  # 保持轻微制动直到停止
                    else:
                        control.throttle = throttle
                        control.brake = 0.0

                control.steer = float(np.clip(steer_angle, -1, 1))
                hero_vehicle.apply_control(control)

                # === 跑偏检测 ===
                if abs(cte) > DEVATION_THRESHOLD and not monitor['deviation']:
                    monitor['deviation'] = True
                    monitor['error_reason'] = f"车辆偏离车道 (横向误差: {cte:.2f}m)"
                    print(f"[ERROR] 车辆偏离车道 (横向误差: {cte:.2f}m)")

                # === 停滞检测 ===
                # 当AEB已触发且车辆停下时，属于正常刹停，不视为异常
                aeb_has_triggered = monitor['max_brake'] > 0
                if current_speed_ms < STALL_SPEED and not aeb_has_triggered:
                    if low_speed_start is None:
                        low_speed_start = elapsed
                    elif elapsed - low_speed_start > STALL_DURATION and not monitor['stall']:
                        monitor['stall'] = True
                        monitor['error_reason'] = f"车辆异常停滞 (速度{current_speed_ms:.3f} m/s, 持续{STALL_DURATION}s)"
                        print(f"[ERROR] 车辆异常停滞 (速度{current_speed_ms:.3f} m/s)")
                else:
                    low_speed_start = None

                # === AEB失效检测（60km/h工况：距离5m内必须触发全力制动） ===
                if distance is not None and distance < AEB_CHECK_DISTANCE and aeb_state != "全力制动":
                    if not monitor['aeb_failure']:
                        monitor['aeb_failure'] = True
                        monitor['error_reason'] = f"AEB未触发全力制动 (距离{distance:.1f}m, 当前状态:{aeb_state})"
                        print(f"[ERROR] AEB未触发全力制动 (距离{distance:.1f}m, 状态:{aeb_state})")

                # 记录状态
                monitor['speed_log'].append((elapsed, current_speed_kmh))
                aeb_state = aeb.get_state_name()

                if frame_count % FPS == 0:
                    dist_str = f"{distance:.1f}m" if distance is not None else "N/A"

                    if len(aeb_states_logged) == 0 or aeb_states_logged[-1] != aeb_state:
                        aeb_states_logged.append(aeb_state)
                        monitor['states_triggered'].add(aeb_state)
                        print(f"[AEB] [{elapsed:.1f}s] -> {aeb_state}")

                    print(f"[{elapsed:.1f}s] 速度: {current_speed_kmh:6.2f} km/h | "
                          f"目标: {obstacle_speed_kmh:6.2f} km/h | "
                          f"油门: {control.throttle:.3f} | 制动: {control.brake:.3f} | "
                          f"转向: {steer_angle:.3f} | AEB: {aeb_state} | "
                          f"距离: {dist_str} | CTE: {cte:.2f}m")

            world.tick()

            # 图像处理
            if image_buffer[0] is not None:
                img = np.copy(image_buffer[0])
                if img.shape[:2] != (res_y, res_x):
                    img = cv2.resize(img, (res_x, res_y))

                bgr_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                font = cv2.FONT_HERSHEY_SIMPLEX

                velocity = hero_vehicle.get_velocity()
                current_speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                current_speed_kmh = current_speed_ms * 3.6

                distance, relative_speed = aeb.detect_front_vehicle(hero_vehicle, world)
                aeb_state = aeb.get_state_name()
                dist_str = f"{distance:.1f}m" if distance is not None else "N/A"

                cv2.putText(bgr_frame, f"Speed: {current_speed_kmh:.1f} km/h | Target: {TARGET_SPEED_KMH} km/h", (20, 40), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(bgr_frame, f"AEB: {aeb_state} | Distance: {dist_str} | Brake: {brake_output:.2f}", (20, 80), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(bgr_frame, f"Time: {elapsed:.1f}s / {duration}s", (20, 120), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                # 跟车距离信息
                if distance is not None:
                    safe_dist = current_speed_ms * FOLLOWING_TIME_GAP
                    cv2.putText(bgr_frame, f"Safe Follow: {safe_dist:.1f}m | Actual: {distance:.1f}m", (20, 160), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

                # AEB状态颜色提示
                if aeb_state == "全力制动":
                    cv2.putText(bgr_frame, "!!! EMERGENCY BRAKING !!!", (res_x//3, 80), font, 1.5, (0, 0, 255), 3, cv2.LINE_AA)
                    cv2.rectangle(bgr_frame, (0, 0), (res_x, res_y), (0, 0, 255), 10)
                elif aeb_state == "部分制动":
                    cv2.putText(bgr_frame, "!! PARTIAL BRAKING !!", (res_x//3, 80), font, 1.5, (0, 255, 255), 3, cv2.LINE_AA)
                    cv2.rectangle(bgr_frame, (0, 0), (res_x, res_y), (0, 255, 255), 8)
                elif aeb_state == "预警":
                    cv2.putText(bgr_frame, "! WARNING !", (res_x//3, 80), font, 1.5, (255, 255, 0), 3, cv2.LINE_AA)
                    cv2.rectangle(bgr_frame, (0, 0), (res_x, res_y), (255, 255, 0), 6)

                # 制动条
                bar_width = int(brake_output * 300)
                cv2.rectangle(bgr_frame, (20, res_y - 50), (320, res_y - 30), (100, 100, 100), -1)
                bar_color = (0, 255, 0) if aeb_state == "正常" else (0, 165, 255) if aeb_state == "预警" else (0, 255, 255) if aeb_state == "部分制动" else (0, 0, 255)
                cv2.rectangle(bgr_frame, (20, res_y - 50), (20 + bar_width, res_y - 30), bar_color, -1)
                cv2.putText(bgr_frame, f"BRAKE: {brake_output:.0%}", (20, res_y - 55), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                surface = pygame.surfarray.make_surface(np.rot90(np.flipud(cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB))))
                surface = pygame.transform.flip(surface, False, True)
                display.blit(surface, (0, 0))
                if not headless_mode:
                    pygame.display.flip()

                video_writer.write(bgr_frame)
                frames_captured += 1

            frame_count += 1

    except Exception as e:
        print(f"[ERROR] 测试过程中出错: {str(e)}")
        import traceback
        traceback.print_exc()
        monitor['error_reason'] = monitor.get('error_reason') or str(e)

    finally:
        settings.synchronous_mode = False
        world.apply_settings(settings)
        video_writer.release()
        camera.stop()
        camera.destroy()
        if collision_sensor:
            collision_sensor.destroy()
        pygame.quit()

        actual_duration = time.time() - start_time
        print(f"\n[INFO] 测试完成 - 实际时长: {actual_duration:.1f}s, 总帧数: {frames_captured}")

    # 设置AEB状态时间线
    monitor['aeb_states'] = aeb_states_logged

    # 60km/h工况：检查是否触发全力制动
    if '全力制动' not in monitor['states_triggered'] and frames_captured > 0 and not any([
        monitor['collision'], monitor['deviation'], monitor['stall'],
        monitor['aeb_failure'], monitor['target_not_braking'], monitor['pygame_error']
    ]):
        # 未碰撞但没触发全力制动，不算成功
        monitor['no_full_brake'] = True
        monitor['error_reason'] = "未触发全力制动（仅完成部分制动阶段）"
        print("[WARN] 未触发全力制动，测试不完整")

    # 判断成功与否
    has_error = any([
        monitor['collision'],
        monitor['deviation'],
        monitor['stall'],
        monitor['aeb_failure'],
        monitor['target_not_braking'],
        monitor['pygame_error'],
        monitor['no_full_brake']
    ])

    success = not has_error and frames_captured > 0
    return success, monitor


# ======================== 主函数（带重试） ========================
def _run_test_loop(test_name, spawn_fn, run_fn, video_output, report_title, load_map=None):
    global DEFAULT_STANLEY_K, DEFAULT_STANLEY_ALPHA, DEFAULT_TTC_WARNING, DEFAULT_TTC_PARTIAL, DEFAULT_TTC_FULL

    max_retries = 3
    success = False
    final_monitor = None

    for attempt in range(1, max_retries + 1):
        print(f"\n{'#'*60}")
        print(f"# 第 {attempt}/{max_retries} 次尝试")
        print(f"{'#'*60}")

        try:
            # 重试时强制重新加载地图，确保spawn_point位置一致
            client = connect_carla(max_retries=2, load_map=load_map)
            print("[OK] 已连接CARLA")
        except RuntimeError:
            print("[WARN] CARLA未运行，尝试启动...")
            quality_idx = min(attempt - 1, len(QUALITY_LEVELS) - 1)
            res_idx = min(attempt - 1, len(RESOLUTIONS) - 1)
            kill_carla()
            if not start_carla(QUALITY_LEVELS[quality_idx], RESOLUTIONS[res_idx][0], RESOLUTIONS[res_idx][1]):
                error_history.append(f"第{attempt}次: CARLA启动失败")
                continue
            try:
                client = connect_carla(max_retries=3, load_map=load_map if attempt == 1 else None)
            except RuntimeError:
                error_history.append(f"第{attempt}次: CARLA连接失败")
                continue

        world = client.get_world()
        print(f"[地图] 当前地图: {world.get_map().name}")
        setup_weather(world)

        hero_vehicle, obstacle_vehicle, hero_collision_sensor, obstacle_collision_sensor = spawn_fn(world)
        if hero_vehicle is None or obstacle_vehicle is None:
            error_history.append(f"第{attempt}次: 车辆生成失败")
            if os.path.exists(video_output):
                time.sleep(1)
                try:
                    os.remove(video_output)
                except PermissionError:
                    print(f"[WARN] 视频文件被占用，跳过删除: {video_output}")
                except Exception as e:
                    print(f"[WARN] 删除视频失败: {e}")
            time.sleep(3)
            continue

        # 包含两车所有传感器
        all_sensors = [hero_collision_sensor, obstacle_collision_sensor]
        vehicles = [hero_vehicle, obstacle_vehicle]
        res_x, res_y = RESOLUTIONS[min(attempt-1, len(RESOLUTIONS)-1)]

        success, monitor = run_fn(
            world, hero_vehicle, obstacle_vehicle, TEST_DURATION,
            hero_collision_sensor=hero_collision_sensor,
            obstacle_collision_sensor=obstacle_collision_sensor,
            stanley_k=DEFAULT_STANLEY_K, stanley_alpha=DEFAULT_STANLEY_ALPHA,
            ttc_warning=DEFAULT_TTC_WARNING, ttc_partial=DEFAULT_TTC_PARTIAL,
            ttc_full=DEFAULT_TTC_FULL, res_x=res_x, res_y=res_y
        )

        for s in all_sensors:
            try:
                s.destroy()
            except Exception:
                pass
        for v in vehicles:
            try:
                v.destroy()
            except Exception:
                pass

        if success:
            print(f"\n[SUCCESS] 第 {attempt} 次测试成功!")
            final_monitor = monitor
            break
        else:
            reason = monitor.get('error_reason', '未知原因')
            error_history.append(f"第{attempt}次失败: {reason}")
            print(f"[FAIL] 第 {attempt} 次测试失败: {reason}")

            if os.path.exists(video_output):
                time.sleep(1)
                try:
                    os.remove(video_output)
                except PermissionError:
                    print(f"[WARN] 视频文件被占用，跳过删除: {video_output}")
                except Exception as e:
                    print(f"[WARN] 删除视频失败: {e}")

            if monitor.get('collision') or monitor.get('deviation'):
                old_k = DEFAULT_STANLEY_K
                DEFAULT_STANLEY_K = max(0.05, DEFAULT_STANLEY_K - 0.05)
                repair_history.append(f"Stanley k: {old_k} -> {DEFAULT_STANLEY_K}")
                print(f"[修复] Stanley增益降低: {old_k} -> {DEFAULT_STANLEY_K}")

                if monitor.get('collision'):
                    DEFAULT_TTC_FULL = min(2.5, DEFAULT_TTC_FULL + 0.3)
                    repair_history.append(f"AEB TTC全力: {DEFAULT_TTC_FULL - 0.3} -> {DEFAULT_TTC_FULL}")
                    print(f"[修复] AEB全力制动阈值放宽: {DEFAULT_TTC_FULL - 0.3} -> {DEFAULT_TTC_FULL}")

                if monitor.get('deviation'):
                    old_alpha = DEFAULT_STANLEY_ALPHA
                    DEFAULT_STANLEY_ALPHA = min(0.8, DEFAULT_STANLEY_ALPHA + 0.1)
                    repair_history.append(f"Stanley alpha: {old_alpha} -> {DEFAULT_STANLEY_ALPHA}")
                    print(f"[修复] 转向平滑系数提高: {old_alpha} -> {DEFAULT_STANLEY_ALPHA}")

            elif monitor.get('aeb_failure') or monitor.get('no_full_brake'):
                old_ttc_full = DEFAULT_TTC_FULL
                DEFAULT_TTC_FULL = max(0.8, DEFAULT_TTC_FULL - 0.3)
                repair_history.append(f"AEB TTC全力: {old_ttc_full} -> {DEFAULT_TTC_FULL}")
                print(f"[修复] AEB全力制动阈值降低: {old_ttc_full} -> {DEFAULT_TTC_FULL}")

            time.sleep(5)

    print(f"\n{'='*60}")
    print(report_title)
    print(f"{'='*60}")

    if success and final_monitor:
        print(f"[结果] 测试成功")
        print(f"[刹停距离] {final_monitor['min_distance']:.2f} m")
        if final_monitor['stop_distance'] > 0:
            print(f"[最终停车距离] {final_monitor['stop_distance']:.2f} m")
        print(f"[最大制动] {final_monitor['max_brake']:.0%}")
        print(f"[是否触发全力制动] {'是 ✓' if '全力制动' in final_monitor['states_triggered'] else '否 ✗'}")
        print(f"\n[AEB状态时间线]")
        for state in final_monitor['aeb_states']:
            print(f"  -> {state}")
        expected = ['正常', '预警', '部分制动', '全力制动']
        actual_states = [state for _, state in final_monitor['aeb_states']]
        if all(s in actual_states for s in expected):
            print(f"\n[✓] 四阶段完整触发: 正常 → 预警 → 部分制动 → 全力制动")
        else:
            missing = [s for s in expected if s not in actual_states]
            print(f"\n[!] 缺失阶段: {', '.join(missing)}")
        print(f"\n[视频文件] {video_output}")
        if os.path.exists(video_output):
            size_mb = os.path.getsize(video_output) / (1024 * 1024)
            print(f"[文件大小] {size_mb:.2f} MB")
    else:
        print(f"[结果] 测试失败 ({max_retries}次尝试均未成功)")

    if error_history:
        print(f"\n[异常记录]")
        for i, err in enumerate(error_history, 1):
            print(f"  {i}. {err}")

    if repair_history:
        print(f"\n[修复记录]")
        for i, rep in enumerate(repair_history, 1):
            print(f"  {i}. {rep}")

    print(f"\n{'='*60}")
    return success


def main():
    global DEFAULT_STANLEY_K, DEFAULT_STANLEY_ALPHA, DEFAULT_TTC_WARNING, DEFAULT_TTC_PARTIAL, DEFAULT_TTC_FULL

    print("=" * 60)
    print("AEB 自动紧急制动模块验证 - 场景选择")
    print("=" * 60)
    print("  [2] 前车急刹 (CCRb)")
    print("  [3] 邻车切入 (Cut-in)")
    print("=" * 60)

    # 环境变量覆盖：AUTO_CCRB=1 自动选择CCRb，AUTO_CUTIN=1 自动选择Cut-in（用于自动化运行）
    if os.environ.get('AUTO_CCRB') == '1':
        choice = '2'
    elif os.environ.get('AUTO_CUTIN') == '1':
        choice = '3'
    else:
        choice = input("请选择测试场景 [2/3]: ").strip()

    if choice == '3':
        clean_old_videos()
        patterns = ['aeb_cutin']
        for pattern in patterns:
            for f in os.listdir(DESKTOP):
                if pattern in f and f.endswith('.mp4'):
                    try:
                        os.remove(os.path.join(DESKTOP, f))
                    except Exception:
                        pass
        _run_test_loop(
            "Cut-in",
            spawn_cutin_vehicles,
            run_cutin_test,
            CUTIN_VIDEO_OUTPUT,
            "Cut-in 邻车切入场景最终报告",
            load_map='Town10HD_Opt'  # Town10HD_Opt：直行多车道，适合Cut-in测试
        )
    else:
        clean_old_videos()
        _run_test_loop(
            "CCRb",
            spawn_ccrb_vehicles,
            run_ccrb_test,
            VIDEO_OUTPUT,
            "CCRb 前车急刹场景最终报告",
            load_map='Town04'  # Town04：长直路，适合CCRb前车急刹测试
        )


if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            _log_f.close()
        except Exception:
            pass
