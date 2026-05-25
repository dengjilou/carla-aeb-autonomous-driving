# -*- coding: utf-8 -*-
"""
CARLA 0.9.13 Stanley横向 + PID纵向控制系统演示
展示：雨天傍晚 + Stanley车道保持 + PID速度控制 + 10辆自动驾驶车辆
"""

import carla
import pygame
import cv2
import numpy as np
import json
import time
import os
import sys
import math
from datetime import datetime
from simple_pid import PID


# ==================== 配置参数 ====================
CARLA_HOST = 'localhost'
CARLA_PORT = 2000
CARLA_TIMEOUT = 5.0
RECORD_DURATION = 90  # 录制时长（秒）
TARGET_SPEED_KMH = 30  # 目标速度 km/h
TARGET_SPEED_MS = TARGET_SPEED_KMH / 3.6  # 转换为 m/s (约8.33)
FPS = 24
VIDEO_OUTPUT = r'C:\Users\22317\OneDrive\Desktop\stanley_pid_demo.mp4'
SCENE_JSON = r'C:\Users\22317\OneDrive\Desktop\zhipu_test_results\result_01.json'
REPORT_FILE = r'C:\Users\22317\OneDrive\Desktop\final_record_report.txt'

# PID纵向控制参数
PID_KP = 0.4
PID_KI = 0.08
PID_KD = 0.2

# Stanley横向控制参数
STANLEY_GAIN_K = 0.3  # Stanley增益（降低以避免过冲）
MAX_STEER_ANGLE = 0.7  # 最大转向角（限制在0.7避免急转弯）
LOOKAHEAD_DISTANCE = 8.0  # 前视距离（增加到8米使控制更平滑）
LOOKAHEAD_FACTOR = 0.3  # 动态前视系数 = 速度(m/s) × factor

# 天气参数（雨天傍晚）
WEATHER_CONFIG = {
    'cloudiness': 90,
    'precipitation': 70,
    'sun_altitude_angle': 15,
    'fog_density': 40
}

# 报告数据
report_data = {
    'start_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
    'weather_params': WEATHER_CONFIG.copy(),
    'pid_params': {'Kp': PID_KP, 'Ki': PID_KI, 'Kd': PID_KD},
    'stanley_gain': STANLEY_GAIN_K,
    'target_speed_kmh': TARGET_SPEED_KMH,
    'vehicle_count': 0,
    'video_resolution': '',
    'video_size_mb': 0,
    'record_duration': 0,
    'success': False,
    'errors': []
}


def log_error(error_msg):
    """记录错误到报告"""
    print(f"[ERROR] {error_msg}")
    report_data['errors'].append(error_msg)


def load_scene_config():
    """加载场景JSON配置"""
    try:
        with open(SCENE_JSON, 'r', encoding='utf-8') as f:
            config = json.load(f)
        vehicle_count = config.get('parsed_json', {}).get('vehicleCount', 10)
        print(f"[INFO] 从JSON读取车辆数量: {vehicle_count}")
        return vehicle_count
    except Exception as e:
        log_error(f"加载场景JSON失败: {str(e)}")
        return 10


def connect_carla():
    """连接CARLA服务器"""
    try:
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(CARLA_TIMEOUT)
        version = client.get_server_version()
        print(f"[SUCCESS] CARLA连接成功 - 版本: {version}")
        return client
    except Exception as e:
        log_error(f"CARLA连接失败: {str(e)}")
        raise


def setup_weather(world):
    """设置雨天傍晚天气"""
    try:
        weather = world.get_weather()
        weather.cloudiness = WEATHER_CONFIG['cloudiness']
        weather.precipitation = WEATHER_CONFIG['precipitation']
        weather.sun_altitude_angle = WEATHER_CONFIG['sun_altitude_angle']
        weather.fog_density = WEATHER_CONFIG['fog_density']
        world.set_weather(weather)
        
        world.wait_for_tick()
        time.sleep(1)
        
        print(f"[SUCCESS] 天气设置完成 - 云量:{weather.cloudiness}%, "
              f"降水:{weather.precipitation}%, 太阳角度:{weather.sun_altitude_angle}度")
        return True
    except Exception as e:
        log_error(f"天气设置失败: {str(e)}")
        return False


def spawn_vehicles(world, num_vehicles):
    """生成自动驾驶车辆"""
    vehicles = []
    blueprint_library = world.get_blueprint_library()
    vehicle_bps = blueprint_library.filter('vehicle.*')
    spawn_points = world.get_map().get_spawn_points()
    
    if len(spawn_points) < num_vehicles:
        log_error(f"生成点不足: 需要{num_vehicles}个，只有{len(spawn_points)}个")
        num_vehicles = len(spawn_points)
    
    print(f"[INFO] 开始生成 {num_vehicles} 辆自动驾驶车辆...")
    
    for i in range(num_vehicles):
        try:
            spawn_point = spawn_points[i]
            vehicle_bp = np.random.choice(vehicle_bps)
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            vehicle.set_autopilot(True)
            vehicles.append(vehicle)
            print(f"  [OK] 车辆 {i+1}/{num_vehicles} 已生成 (ID: {vehicle.id})")
        except Exception as e:
            log_error(f"车辆 {i+1} 生成失败: {str(e)}")
    
    print("[INFO] 等待3秒让物理引擎稳定...")
    time.sleep(3)
    
    report_data['vehicle_count'] = len(vehicles)
    print(f"[SUCCESS] 共生成 {len(vehicles)} 辆车辆")
    return vehicles


class AEBController:
    """AEB自动紧急制动控制器"""
    
    def __init__(self, ttc_warning=2.5, ttc_partial=1.6, ttc_full=0.6, detection_range=50.0):
        # 状态机状态：0-正常 1-预警 2-部分制动 3-全力制动
        self.NORMAL = 0
        self.WARNING = 1
        self.PARTIAL_BRAKE = 2
        self.FULL_BRAKE = 3
        
        self.state = self.NORMAL  # 当前状态
        self.ttc_warning = ttc_warning  # 预警TTC阈值（秒）
        self.ttc_partial = ttc_partial  # 部分制动TTC阈值（秒）
        self.ttc_full = ttc_full  # 全力制动TTC阈值（秒）
        self.detection_range = detection_range  # 检测范围（米）
        
        # 制动输出参数
        self.warning_brake = 0.0  # 预警时不制动
        self.partial_brake_strength = 0.4  # 部分制动强度（0-1）
        self.full_brake_strength = 1.0  # 全力制动强度（0-1）
    
    def calculate_ttc(self, distance, relative_speed):
        """
        计算碰撞时间（TTC）
        
        参数:
        - distance: 与前车的距离（米）
        - relative_speed: 相对速度（m/s），正数表示接近
        
        返回:
        - TTC（秒），如果无法计算则返回无穷大
        """
        if relative_speed <= 0.1:  # 相对速度过小，视为无碰撞风险
            return float('inf')
        
        return distance / relative_speed
    
    def detect_front_vehicle(self, hero_vehicle, world, detection_range=None, target_velocity=None):
        """
        检测前方车辆，返回最近车辆的距离和相对速度
        
        参数:
        - hero_vehicle: 主角车辆
        - world: CARLA世界对象
        - detection_range: 检测范围（米），使用实例默认值
        - target_velocity: 外部注入的目标车速度（m/s），用于物理关闭时的虚拟速度
        
        返回:
        - distance: 前方最近车辆距离（米），未检测到返回None
        - relative_speed: 相对速度（m/s），未检测到返回0
        """
        try:
            if detection_range is None:
                detection_range = self.detection_range
            
            # 获取本车位置和前进方向
            hero_transform = hero_vehicle.get_transform()
            hero_location = hero_transform.location
            hero_heading = math.radians(hero_transform.rotation.yaw)
            
            # 获取世界中的所有车辆
            vehicle_list = world.get_actors().filter('vehicle.*')
            
            closest_distance = None
            closest_vehicle = None
            
            for vehicle in vehicle_list:
                if vehicle.id == hero_vehicle.id:
                    continue
                
                # 获取目标车辆位置和速度
                vehicle_transform = vehicle.get_transform()
                vehicle_location = vehicle_transform.location
                vehicle_velocity = vehicle.get_velocity()
                
                # 计算相对位置
                dx = vehicle_location.x - hero_location.x
                dy = vehicle_location.y - hero_location.y
                
                # 计算距离
                distance = math.sqrt(dx*dx + dy*dy)
                
                # 如果超出检测范围，跳过
                if distance > detection_range:
                    continue
                
                # 检查是否在本车前方（点积大于0表示在前方）
                dot_product = dx * math.cos(hero_heading) + dy * math.sin(hero_heading)
                if dot_product < 0:
                    continue  # 在后方，跳过
                
                # 检查是否在同一车道附近（横向偏移小）
                cross_product = dx * (-math.sin(hero_heading)) + dy * math.cos(hero_heading)
                if abs(cross_product) > 10.0:  # 放宽到10m以覆盖Cut-in场景（目标车从相邻车道切入）
                    continue
                
                # 调试日志：打印候选车辆信息
                # print(f"  [AEB调试] 候选车辆ID={vehicle.id}, 距离={distance:.1f}m, 横向={abs(cross_product):.1f}m")
                
                # 更新最近车辆
                if closest_distance is None or distance < closest_distance:
                    closest_distance = distance
                    closest_vehicle = vehicle
            
            if closest_vehicle is None:
                return None, 0
            
            # 计算相对速度
            hero_velocity = hero_vehicle.get_velocity()
            hero_speed = math.sqrt(hero_velocity.x**2 + hero_velocity.y**2)
            
            # 目标车辆速度在本车前进方向的分量
            if target_velocity is not None:
                # 使用外部注入的目标车速度（物理关闭期间）
                vehicle_speed_forward = target_velocity
            else:
                vehicle_speed_forward = (vehicle_velocity.x * math.cos(hero_heading) + 
                                         vehicle_velocity.y * math.sin(hero_heading))
            
            # 相对速度（本车速度 - 前车速度，正数表示接近）
            relative_speed = hero_speed - vehicle_speed_forward
            
            return closest_distance, relative_speed
            
        except Exception as e:
            log_error(f"AEB前方检测错误: {str(e)}")
            return None, 0
    
    def update_state(self, distance, relative_speed):
        """
        更新AEB状态机（单向锁定：只能升级不能降级）
        
        参数:
        - distance: 前方车辆距离（米）
        - relative_speed: 相对速度（m/s）
        
        返回:
        - brake_output: 制动输出（0-1）
        """
        # 如果未检测到前方车辆，恢复为正常状态
        if distance is None:
            self.state = self.NORMAL
            return 0.0
        
        # 计算TTC
        ttc = self.calculate_ttc(distance, relative_speed)
        
        # 根据当前TTC计算应该进入的等级
        if ttc <= self.ttc_full:
            target_state = self.FULL_BRAKE
        elif ttc <= self.ttc_partial:
            target_state = self.PARTIAL_BRAKE
        elif ttc <= self.ttc_warning:
            target_state = self.WARNING
        else:
            target_state = self.NORMAL
        
        # 距离紧急保护：当距离过近时强制升级（防止相对速度=0时TTC=∞导致无法升级）
        if distance < 10.0 and self.state < self.FULL_BRAKE:
            target_state = self.FULL_BRAKE
        elif distance < 15.0 and self.state < self.PARTIAL_BRAKE:
            target_state = self.PARTIAL_BRAKE
        
        # 单向锁定：只允许状态升级，不允许降级
        if target_state > self.state:
            self.state = target_state
        
        # 根据当前状态输出制动
        if self.state == self.FULL_BRAKE:
            return self.full_brake_strength
        elif self.state == self.PARTIAL_BRAKE:
            return self.partial_brake_strength
        elif self.state == self.WARNING:
            return self.warning_brake
        else:
            return 0.0
    
    def get_state_name(self):
        """获取当前状态名称"""
        state_names = {
            self.NORMAL: "正常",
            self.WARNING: "预警",
            self.PARTIAL_BRAKE: "部分制动",
            self.FULL_BRAKE: "全力制动"
        }
        return state_names.get(self.state, "未知")


class StanleyController:
    """Stanley横向控制器实现"""
    
    def __init__(self, k=STANLEY_GAIN_K, max_steer=MAX_STEER_ANGLE, lookahead_factor=LOOKAHEAD_FACTOR):
        self.k = k  # Stanley增益
        self.max_steer = max_steer  # 最大转向角
        self.prev_steer = 0.0  # 上一次转向角（用于平滑）
        self.steer_alpha = 0.4  # 转向平滑系数（0.4平衡响应与稳定）
        self.lookahead_factor = lookahead_factor  # 动态前视系数 = 速度(m/s) × factor
        self.min_lookahead = 3.0  # 最小前视距离
        self.max_lookahead = 15.0  # 最大前视距离
    
    def compute_steering_angle(self, vehicle, waypoint):
        """
        计算Stanley转向角
        
        公式: δ = θ_e + arctan(k * e / v)
        其中:
        - θ_e: 航向误差（目标航向 - 当前航向）
        - e: 横向误差（车辆到目标路径的垂直距离）
        - v: 当前车速
        - k: Stanley增益
        
        返回: 归一化的转向角 (-1 到 1)
        """
        try:
            # 获取车辆状态
            transform = vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation
            
            # 车辆当前位置
            vehicle_x = location.x
            vehicle_y = location.y
            
            # 车辆当前航向（转换为弧度）
            vehicle_heading = math.radians(rotation.yaw)
            
            # 目标waypoint信息
            wp_location = waypoint.transform.location
            wp_rotation = waypoint.transform.rotation
            
            # 目标位置
            target_x = wp_location.x
            target_y = wp_location.y
            
            # 目标航向
            target_heading = math.radians(wp_rotation.yaw)
            
            # 计算航向误差 θ_e
            heading_error = target_heading - vehicle_heading
            # 将航向误差限制在 [-π, π]
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
            
            # 计算横向误差 e
            # 使用向量叉乘计算真正的横向误差
            dx = target_x - vehicle_x
            dy = target_y - vehicle_y
            
            # 车辆前进方向的单位向量
            vehicle_forward_x = math.cos(vehicle_heading)
            vehicle_forward_y = math.sin(vehicle_heading)
            
            # 横向误差 = 向量叉乘（垂直于车辆方向的距离）
            cross_track_error = dx * (-vehicle_forward_y) + dy * vehicle_forward_x
            
            # 获取当前车速
            velocity = vehicle.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            # 防止除零
            if speed < 0.1:
                speed = 0.1
            
            # Stanley公式: δ = θ_e + arctan(k * e / v)
            raw_steer = heading_error + math.atan2(self.k * cross_track_error, speed)
            
            # 限制原始转向角
            raw_steer = max(-self.max_steer, min(self.max_steer, raw_steer))
            
            # 应用低通滤波使转向更平滑
            steer_angle = self.steer_alpha * raw_steer + (1 - self.steer_alpha) * self.prev_steer
            
            # 再次限制最终转向角
            steer_angle = max(-self.max_steer, min(self.max_steer, steer_angle))
            
            # 更新上一次转向角
            self.prev_steer = steer_angle
            
            return steer_angle, cross_track_error, heading_error, speed
            
        except Exception as e:
            log_error(f"Stanley控制器计算错误: {str(e)}")
            return 0.0, 0.0, 0.0, 0.0


def get_target_waypoint(world, vehicle, lookahead_distance=LOOKAHEAD_DISTANCE, min_lookahead=3.0, max_lookahead=15.0, speed_factor=None):
    """
    获取目标waypoint
    
    参数:
    - world: CARLA世界对象
    - vehicle: 车辆对象
    - lookahead_distance: 固定前视距离（米）
    - min_lookahead: 最小前视距离
    - max_lookahead: 最大前视距离
    - speed_factor: 动态前视系数（速度×factor），为None时使用固定距离
    """
    try:
        map = world.get_map()
        location = vehicle.get_location()
        
        # 获取当前位置的waypoint
        current_waypoint = map.get_waypoint(location)
        
        # 计算实际前视距离
        actual_lookahead = lookahead_distance
        if speed_factor is not None:
            velocity = vehicle.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            actual_lookahead = speed * speed_factor
            actual_lookahead = max(min_lookahead, min(max_lookahead, actual_lookahead))
        
        # 沿道路方向前进lookahead_distance获取目标waypoint
        next_waypoints = current_waypoint.next(actual_lookahead)
        
        if next_waypoints:
            return next_waypoints[0]
        else:
            return current_waypoint
    except Exception as e:
        log_error(f"获取waypoint失败: {str(e)}")
        return None


def record_video(world, hero_vehicle, vehicles, duration):
    """录制视频主函数"""
    print(f"\n[INFO] 开始录制 {duration} 秒视频...")
    print(f"[INFO] 目标速度: {TARGET_SPEED_KMH} km/h")
    print(f"[INFO] Stanley增益: {STANLEY_GAIN_K}")
    print("-" * 60)
    
    # 初始化PID纵向控制器
    pid_lon = PID(PID_KP, PID_KI, PID_KD, setpoint=TARGET_SPEED_MS)
    pid_lon.output_limits = (0, 1)  # 油门范围 0-1
    print(f"[SUCCESS] PID纵向控制器初始化 - Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
    
    # 初始化Stanley横向控制器
    stanley = StanleyController(k=STANLEY_GAIN_K, max_steer=MAX_STEER_ANGLE)
    print(f"[SUCCESS] Stanley横向控制器初始化 - k={STANLEY_GAIN_K}")
    
    # 初始化AEB自动紧急制动控制器
    aeb = AEBController(ttc_warning=2.5, ttc_partial=1.6, ttc_full=0.6, detection_range=50.0)
    print(f"[SUCCESS] AEB控制器初始化 - 预警:{aeb.ttc_warning}s | 部分制动:{aeb.ttc_partial}s | 全力制动:{aeb.ttc_full}s")
    
    # 启用同步模式
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / FPS
    world.apply_settings(settings)
    
    # 创建RGB摄像头
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')
    camera_bp.set_attribute('fov', '90')
    
    camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=hero_vehicle)
    
    # 初始化Pygame
    pygame.init()
    display = pygame.display.set_mode((1920, 1080), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption('CARLA Stanley + PID Control Demo - Rainy Evening')
    
    # 图像缓冲区
    image_buffer = [None]
    
    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # 去掉alpha通道
        array = array[:, :, ::-1]  # BGRA to RGB
        image_buffer[0] = array
    
    camera.listen(camera_callback)
    
    # 初始化视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(VIDEO_OUTPUT, fourcc, FPS, (1920, 1080))
    
    if not video_writer.isOpened():
        log_error("VideoWriter初始化失败")
        camera.stop()
        camera.destroy()
        pygame.quit()
        return False
    
    # 帧计数和时间控制
    frame_count = 0
    start_time = time.time()
    frames_captured = 0
    
    frame_30_saved = False
    frame_60_saved = False
    
    try:
        running = True
        while running:
            current_time = time.time()
            elapsed = current_time - start_time
            
            if elapsed >= duration:
                break
            
            # 处理Pygame事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
            
            if not running:
                break
            
            # 获取目标waypoint
            target_waypoint = get_target_waypoint(world, hero_vehicle, LOOKAHEAD_DISTANCE)
            
            if target_waypoint is not None:
                # Stanley横向控制
                steer_angle, cte, heading_err, speed = stanley.compute_steering_angle(
                    hero_vehicle, target_waypoint
                )
                
                # PID纵向控制
                velocity = hero_vehicle.get_velocity()
                current_speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                current_speed_kmh = current_speed_ms * 3.6
                
                throttle = pid_lon(current_speed_ms)
                throttle = float(np.clip(throttle, 0, 1))
                
                # AEB自动紧急制动：检测前方车辆并决策
                distance, relative_speed = aeb.detect_front_vehicle(hero_vehicle, world)
                brake_output = aeb.update_state(distance, relative_speed)
                
                # 应用控制：AEB优先于PID
                control = carla.VehicleControl()
                if brake_output > 0:
                    # AEB触发：覆盖油门，输出制动
                    control.throttle = 0.0
                    control.brake = brake_output
                else:
                    # AEB未触发：使用PID油门
                    control.throttle = throttle
                    control.brake = 0.0
                
                control.steer = float(np.clip(steer_angle, -1, 1))
                hero_vehicle.apply_control(control)
                
                # 实时打印状态（每秒一次）
                if frame_count % FPS == 0:
                    aeb_status = aeb.get_state_name()
                    dist_str = f"{distance:.1f}m" if distance is not None else "N/A"
                    print(f"[{elapsed:.1f}s] 速度: {current_speed_kmh:6.2f} km/h | "
                          f"油门: {control.throttle:.3f} | 制动: {control.brake:.3f} | "
                          f"转向: {steer_angle:.3f} | AEB: {aeb_status} | "
                          f"前车距离: {dist_str}m")
            
            # CARLA tick
            world.tick()
            
            # 检查是否有新图像
            if image_buffer[0] is not None:
                img = np.copy(image_buffer[0])
                
                if img.shape[:2] != (1080, 1920):
                    img = cv2.resize(img, (1920, 1080))
                
                # 转换为BGR用于OpenCV
                bgr_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                
                # 叠加文字信息
                font = cv2.FONT_HERSHEY_SIMPLEX
                info_text = f"Speed: {current_speed_kmh:.1f} km/h | Target: {TARGET_SPEED_KMH} km/h"
                cv2.putText(bgr_frame, info_text, (20, 40), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                
                control_text = f"Throttle: {throttle:.2f} | Steer: {steer_angle:.2f} | CTE: {cte:.2f}m"
                cv2.putText(bgr_frame, control_text, (20, 80), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                
                time_text = f"Time: {elapsed:.1f}s / {duration}s"
                cv2.putText(bgr_frame, time_text, (20, 120), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                
                # 转换回RGB用于Pygame
                rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
                
                # 显示
                surface = pygame.surfarray.make_surface(np.rot90(np.flipud(rgb_frame)))
                display.blit(surface, (0, 0))
                pygame.display.flip()
                
                # 写入视频
                video_writer.write(bgr_frame)
                frames_captured += 1
                
                # 保存第30帧和第60帧
                if frames_captured == 30 and not frame_30_saved:
                    cv2.imwrite(r'C:\Users\22317\OneDrive\Desktop\frame_30.jpg', bgr_frame)
                    frame_30_saved = True
                    print(f"[INFO] 已保存第30帧截图")
                
                if frames_captured == 60 and not frame_60_saved:
                    cv2.imwrite(r'C:\Users\22317\OneDrive\Desktop\frame_60.jpg', bgr_frame)
                    frame_60_saved = True
                    print(f"[INFO] 已保存第60帧截图")
            
            frame_count += 1
    
    except Exception as e:
        log_error(f"录制过程中出错: {str(e)}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 恢复异步模式
        settings.synchronous_mode = False
        world.apply_settings(settings)
        
        # 清理资源
        video_writer.release()
        camera.stop()
        camera.destroy()
        pygame.quit()
        
        actual_duration = time.time() - start_time
        report_data['record_duration'] = actual_duration
        report_data['video_resolution'] = '1920x1080'
        
        print(f"\n[INFO] 录制完成 - 实际时长: {actual_duration:.1f}s, 总帧数: {frames_captured}")
    
    return True


def verify_video():
    """验证视频文件"""
    if not os.path.exists(VIDEO_OUTPUT):
        log_error("视频文件不存在")
        return False
    
    file_size = os.path.getsize(VIDEO_OUTPUT)
    file_size_mb = file_size / (1024 * 1024)
    report_data['video_size_mb'] = round(file_size_mb, 2)
    
    print(f"\n[INFO] 视频文件大小: {file_size_mb:.2f} MB")
    
    cap = cv2.VideoCapture(VIDEO_OUTPUT)
    if cap.isOpened():
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        
        print(f"[INFO] 视频信息 - 分辨率: {width}x{height}, FPS: {fps}, 总帧数: {frame_count}")
        report_data['video_resolution'] = f'{width}x{height}'
    
    return file_size_mb > 10


def generate_report():
    """生成执行报告"""
    report_data['end_time'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    report_data['success'] = os.path.exists(VIDEO_OUTPUT)
    
    with open(REPORT_FILE, 'w', encoding='utf-8') as f:
        f.write("=" * 60 + "\n")
        f.write("CARLA Stanley+PID控制系统演示视频 - 执行报告\n")
        f.write("=" * 60 + "\n\n")
        
        f.write(f"开始时间: {report_data['start_time']}\n")
        f.write(f"结束时间: {report_data['end_time']}\n\n")
        
        f.write("--- 天气参数 ---\n")
        for key, value in report_data['weather_params'].items():
            f.write(f"  {key}: {value}\n")
        f.write("\n")
        
        f.write("--- PID纵向控制参数 ---\n")
        for key, value in report_data['pid_params'].items():
            f.write(f"  {key}: {value}\n")
        f.write(f"  目标速度: {report_data['target_speed_kmh']} km/h\n\n")
        
        f.write("--- Stanley横向控制参数 ---\n")
        f.write(f"  增益k: {report_data['stanley_gain']}\n")
        f.write(f"  最大转向角: {MAX_STEER_ANGLE}\n\n")
        
        f.write(f"--- 车辆数量: {report_data['vehicle_count']} ---\n\n")
        
        f.write("--- 视频信息 ---\n")
        f.write(f"  输出路径: {VIDEO_OUTPUT}\n")
        f.write(f"  分辨率: {report_data['video_resolution']}\n")
        f.write(f"  文件大小: {report_data['video_size_mb']} MB\n")
        f.write(f"  录制时长: {report_data['record_duration']:.1f} 秒\n\n")
        
        f.write(f"--- 执行结果 ---\n")
        f.write(f"  是否成功: {'是' if report_data['success'] else '否'}\n\n")
        
        if report_data['errors']:
            f.write("--- 错误记录 ---\n")
            for i, error in enumerate(report_data['errors'], 1):
                f.write(f"  {i}. {error}\n")
            f.write("\n")
        
        f.write("=" * 60 + "\n")
    
    print(f"\n[SUCCESS] 执行报告已保存: {REPORT_FILE}")


def cleanup(vehicles, world):
    """清理资源"""
    print("\n[INFO] 清理资源...")
    for vehicle in vehicles:
        try:
            vehicle.destroy()
        except:
            pass
    print("[INFO] 所有车辆已销毁")


def main():
    """主函数"""
    print("=" * 60)
    print("CARLA 0.9.13 Stanley+PID控制系统演示视频录制")
    print("=" * 60)
    print(f"目标: 雨天傍晚 + Stanley横向控制 + PID纵向控制 + 10辆自动驾驶车辆")
    print(f"录制时长: {RECORD_DURATION} 秒")
    print(f"输出路径: {VIDEO_OUTPUT}")
    print("=" * 60)
    
    vehicles = []
    world = None
    
    try:
        # 1. 加载场景配置
        num_vehicles = load_scene_config()
        
        # 2. 连接CARLA
        client = connect_carla()
        world = client.get_world()
        
        # 3. 设置天气
        setup_weather(world)
        
        # 4. 生成车辆
        vehicles = spawn_vehicles(world, num_vehicles)
        
        if not vehicles:
            log_error("没有成功生成任何车辆")
            return
        
        # 5. 选择主角车辆（第一辆）
        hero_vehicle = vehicles[0]
        print(f"\n[INFO] 主角车辆 ID: {hero_vehicle.id}")
        
        # 6. 录制视频
        success = record_video(world, hero_vehicle, vehicles, RECORD_DURATION)
        
        if success:
            verify_video()
        
        # 7. 生成报告
        generate_report()
        
        # 8. 打印摘要
        print("\n" + "=" * 60)
        print("执行结果摘要")
        print("=" * 60)
        print(f"视频路径: {VIDEO_OUTPUT}")
        print(f"文件大小: {report_data['video_size_mb']} MB")
        print(f"分辨率: {report_data['video_resolution']}")
        print(f"录制时长: {report_data['record_duration']:.1f} 秒")
        print(f"车辆数量: {report_data['vehicle_count']}")
        print(f"是否成功: {'是' if report_data['success'] else '否'}")
        print("=" * 60)
    
    except Exception as e:
        log_error(f"主流程异常: {str(e)}")
        import traceback
        traceback.print_exc()
        generate_report()
    
    finally:
        if vehicles and world:
            cleanup(vehicles, world)


if __name__ == "__main__":
    main()
