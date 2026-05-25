# -*- coding: utf-8 -*-
"""
AEB自动紧急制动模块验证脚本 - 无头版（无pygame窗口，适合后台运行）
场景：自车以40km/h驶向正前方30米处的静止目标车
预期：AEB触发，自车减速/刹停避免碰撞
"""

import carla
import cv2
import numpy as np
import math
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from stanley_pid_controller import AEBController, StanleyController, get_target_waypoint
from simple_pid import PID

CARLA_HOST = 'localhost'
CARLA_PORT = 2000
TEST_DURATION = 30
TARGET_SPEED_KMH = 40
TARGET_SPEED_MS = TARGET_SPEED_KMH / 3.6
FPS = 24
VIDEO_OUTPUT = r'C:\Users\22317\OneDrive\Desktop\aeb_ccrb_demo.mp4'
OBSTACLE_DISTANCE = 30

# Stanley控制器参数
STANLEY_GAIN_K = 0.2
STANLEY_STEER_ALPHA = 0.4

WEATHER_CONFIG = {
    'cloudiness': 10,
    'precipitation': 0,
    'sun_altitude_angle': 45,
    'fog_density': 0
}


def clean_old_videos():
    desktop = r'C:\Users\22317\OneDrive\Desktop'
    patterns = ['aeb_ccrb', 'aeb_validation', 'carla_', 'stanley_pid_demo']
    for pattern in patterns:
        for f in os.listdir(desktop):
            if pattern in f and f.endswith('.mp4'):
                try:
                    os.remove(os.path.join(desktop, f))
                    print(f"  已删除旧视频: {f}")
                except Exception:
                    pass


def connect_carla(max_retries=5):
    for attempt in range(max_retries):
        try:
            client = carla.Client(CARLA_HOST, CARLA_PORT)
            client.set_timeout(20.0)
            version = client.get_server_version()
            print(f"[SUCCESS] CARLA连接成功 - 版本: {version}")
            return client
        except RuntimeError as e:
            wait_time = 10 * (attempt + 1)
            print(f"[WARN] CARLA连接失败 (尝试 {attempt+1}/{max_retries}): {str(e)[:80]}")
            print(f"       等待 {wait_time} 秒后重试...")
            time.sleep(wait_time)
    raise RuntimeError("无法连接到CARLA服务器")


def setup_weather(world):
    weather = world.get_weather()
    weather.cloudiness = WEATHER_CONFIG['cloudiness']
    weather.precipitation = WEATHER_CONFIG['precipitation']
    weather.sun_altitude_angle = WEATHER_CONFIG['sun_altitude_angle']
    weather.fog_density = WEATHER_CONFIG['fog_density']
    world.set_weather(weather)
    print("[SUCCESS] 天气设置完成")


def spawn_test_vehicles(world):
    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    if len(spawn_points) < 2:
        print("[ERROR] 生成点不足")
        return None, None

    hero_bp = blueprint_library.find('vehicle.tesla.model3')
    hero_vehicle = None

    for i in range(min(10, len(spawn_points))):
        try:
            hero_spawn = spawn_points[i]
            hero_vehicle = world.spawn_actor(hero_bp, hero_spawn)
            print(f"[OK] 自车已生成 (ID: {hero_vehicle.id})，使用生成点 {i}")
            break
        except Exception:
            continue

    if hero_vehicle is None:
        print("[ERROR] 无法生成自车")
        return None, None

    hero_transform = hero_vehicle.get_transform()
    hero_location = hero_transform.location
    hero_yaw = math.radians(hero_transform.rotation.yaw)

    obstacle_x = hero_location.x + OBSTACLE_DISTANCE * math.cos(hero_yaw)
    obstacle_y = hero_location.y + OBSTACLE_DISTANCE * math.sin(hero_yaw)
    obstacle_z = hero_location.z
    obstacle_yaw = hero_transform.rotation.yaw

    obstacle_location = carla.Location(x=obstacle_x, y=obstacle_y, z=obstacle_z)
    obstacle_rotation = carla.Rotation(yaw=obstacle_yaw)
    obstacle_transform = carla.Transform(obstacle_location, obstacle_rotation)

    obstacle_bp = blueprint_library.find('vehicle.tesla.model3')
    obstacle_vehicle = None

    try:
        obstacle_vehicle = world.spawn_actor(obstacle_bp, obstacle_transform)
        obstacle_vehicle.set_simulate_physics(True)
        print(f"[OK] 目标车已生成 (ID: {obstacle_vehicle.id})，距离自车 {OBSTACLE_DISTANCE} 米")
    except Exception:
        for sp in spawn_points:
            sp_loc = sp.location
            dist = math.sqrt((sp_loc.x - hero_location.x)**2 + (sp_loc.y - hero_location.y)**2)
            if 25 < dist < 45:
                try:
                    obstacle_vehicle = world.spawn_actor(obstacle_bp, sp)
                    obstacle_vehicle.set_simulate_physics(True)
                    actual_dist = math.sqrt((sp_loc.x - hero_location.x)**2 + (sp_loc.y - hero_location.y)**2)
                    print(f"[OK] 目标车已生成 (ID: {obstacle_vehicle.id})，实际距离约 {actual_dist:.1f} 米")
                    break
                except Exception:
                    continue

    if obstacle_vehicle is None:
        print("[ERROR] 无法生成目标车")
        hero_vehicle.destroy()
        return None, None

    time.sleep(2)
    return hero_vehicle, obstacle_vehicle


def run_aeb_test(world, hero_vehicle, obstacle_vehicle, duration):
    print(f"\n[INFO] 开始CCRb场景测试，时长 {duration} 秒")
    print(f"[INFO] 目标速度: {TARGET_SPEED_KMH} km/h")
    print(f"[INFO] 目标车距离: {OBSTACLE_DISTANCE} 米")
    print("-" * 60)

    stanley = StanleyController(k=STANLEY_GAIN_K, max_steer=0.5)
    stanley.steer_alpha = STANLEY_STEER_ALPHA
    aeb = AEBController(ttc_warning=2.5, ttc_partial=1.6, ttc_full=0.6, detection_range=50.0)
    pid_lon = PID(0.4, 0.08, 0.2, setpoint=TARGET_SPEED_MS)
    pid_lon.output_limits = (0, 1)
    print(f"[INFO] Stanley参数: k={STANLEY_GAIN_K}, steer_alpha={STANLEY_STEER_ALPHA}")

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / FPS
    world.apply_settings(settings)

    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')
    camera_bp.set_attribute('fov', '90')

    camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=hero_vehicle)

    image_buffer = [None]

    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        image_buffer[0] = array

    camera.listen(camera_callback)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(VIDEO_OUTPUT, fourcc, FPS, (1920, 1080))

    if not video_writer.isOpened():
        print("[ERROR] VideoWriter初始化失败")
        return False

    frame_count = 0
    start_time = time.time()
    frames_captured = 0
    aeb_state_history = []
    collision_detected = False
    min_distance = 999.0
    final_stopping_distance = 0.0

    try:
        running = True
        while running:
            current_time = time.time()
            elapsed = current_time - start_time

            if elapsed >= duration:
                print(f"\n[INFO] 达到测试时长 {duration} 秒")
                break

            # 获取目标waypoint
            target_waypoint = get_target_waypoint(world, hero_vehicle, 8.0)

            if target_waypoint is not None:
                steer_angle, cte, heading_err, speed = stanley.compute_steering_angle(
                    hero_vehicle, target_waypoint
                )

                velocity = hero_vehicle.get_velocity()
                current_speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                current_speed_kmh = current_speed_ms * 3.6

                throttle = pid_lon(current_speed_ms)
                throttle = float(np.clip(throttle, 0, 1))

                distance, relative_speed = aeb.detect_front_vehicle(hero_vehicle, world)
                brake_output = aeb.update_state(distance, relative_speed)

                # 跟踪最小距离
                if distance is not None:
                    if distance < min_distance:
                        min_distance = distance
                    if current_speed_kmh < 1.0 and distance < min_distance + 1.0:
                        final_stopping_distance = distance

                # 碰撞检测
                if distance is not None and distance < 2.5:
                    collision_detected = True

                control = carla.VehicleControl()
                if brake_output > 0:
                    control.throttle = 0.0
                    control.brake = brake_output
                else:
                    control.throttle = throttle
                    control.brake = 0.0

                control.steer = float(np.clip(steer_angle, -1, 1))
                hero_vehicle.apply_control(control)

                # 记录AEB状态变化
                aeb_state = aeb.get_state_name()
                if frame_count % FPS == 0:
                    dist_str = f"{distance:.1f}m" if distance is not None else "N/A"
                    if len(aeb_state_history) == 0 or aeb_state_history[-1][1] != aeb_state:
                        aeb_state_history.append((elapsed, aeb_state))
                        print(f"[AEB状态变化 @ {elapsed:.1f}s] -> {aeb_state}")

                    print(f"[{elapsed:.1f}s] 速度: {current_speed_kmh:6.2f} km/h | "
                          f"油门: {control.throttle:.3f} | 制动: {control.brake:.3f} | "
                          f"转向: {steer_angle:.3f} | AEB: {aeb_state} | "
                          f"前车距离: {dist_str}")

            world.tick()

            # 图像处理和视频录制（无pygame显示）
            if image_buffer[0] is not None:
                img = np.copy(image_buffer[0])
                if img.shape[:2] != (1080, 1920):
                    img = cv2.resize(img, (1920, 1080))

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

                if aeb_state == "全力制动":
                    cv2.putText(bgr_frame, "!!! EMERGENCY BRAKING !!!", (600, 80), font, 1.5, (0, 0, 255), 3, cv2.LINE_AA)
                elif aeb_state == "部分制动":
                    cv2.putText(bgr_frame, "!! PARTIAL BRAKING !!", (600, 80), font, 1.5, (0, 255, 255), 3, cv2.LINE_AA)
                elif aeb_state == "预警":
                    cv2.putText(bgr_frame, "! WARNING !", (600, 80), font, 1.5, (255, 255, 0), 3, cv2.LINE_AA)

                video_writer.write(bgr_frame)
                frames_captured += 1

            frame_count += 1

    except Exception as e:
        print(f"[ERROR] 测试过程中出错: {str(e)}")
        import traceback
        traceback.print_exc()

    finally:
        settings.synchronous_mode = False
        world.apply_settings(settings)
        video_writer.release()
        camera.stop()
        camera.destroy()

        actual_duration = time.time() - start_time
        print(f"\n[INFO] 测试完成 - 实际时长: {actual_duration:.1f}s, 总帧数: {frames_captured}")

    # 输出最终报告
    print("\n" + "=" * 60)
    print("CCRb 场景测试结果")
    print("=" * 60)
    print(f"是否碰撞: {'是 (碰撞!)' if collision_detected else '否 (成功避免)'}")
    print(f"最小距离: {min_distance:.2f} m" if min_distance < 999 else "最小距离: N/A")
    print(f"刹停距离: {final_stopping_distance:.2f} m" if final_stopping_distance > 0 else "刹停距离: N/A")
    print(f"\nAEB状态变化时间线:")
    for t, state in aeb_state_history:
        print(f"  [{t:.1f}s] -> {state}")

    if os.path.exists(VIDEO_OUTPUT):
        file_size_mb = os.path.getsize(VIDEO_OUTPUT) / (1024 * 1024)
        print(f"\n视频文件: {VIDEO_OUTPUT}")
        print(f"文件大小: {file_size_mb:.2f} MB")
        print(f"录制帧数: {frames_captured}")
    else:
        print("\n[ERROR] 视频文件未生成")

    return True


def cleanup(vehicles, world):
    print("\n[INFO] 清理资源...")
    for vehicle in vehicles:
        try:
            vehicle.destroy()
        except Exception:
            pass
    print("[INFO] 所有车辆已销毁")


def main():
    print("=" * 60)
    print("AEB自动紧急制动模块验证测试 - CCRb场景")
    print("=" * 60)
    print(f"场景: 自车以{TARGET_SPEED_KMH}km/h驶向正前方{OBSTACLE_DISTANCE}米处的静止目标车")
    print(f"预期: AEB触发，自车减速/刹停避免碰撞")
    print(f"视频输出: {VIDEO_OUTPUT}")
    print("=" * 60)

    clean_old_videos()

    vehicles = []
    world = None

    try:
        client = connect_carla()
        world = client.get_world()

        setup_weather(world)

        hero_vehicle, obstacle_vehicle = spawn_test_vehicles(world)

        if hero_vehicle is None or obstacle_vehicle is None:
            print("[ERROR] 车辆生成失败")
            return

        vehicles = [hero_vehicle, obstacle_vehicle]

        success = run_aeb_test(world, hero_vehicle, obstacle_vehicle, TEST_DURATION)

    except Exception as e:
        print(f"[ERROR] 主流程异常: {str(e)}")
        import traceback
        traceback.print_exc()

    finally:
        if vehicles and world:
            cleanup(vehicles, world)


if __name__ == "__main__":
    main()
