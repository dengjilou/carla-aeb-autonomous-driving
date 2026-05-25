#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
自然语言 -> JSON -> CARLA场景自动配置脚本

功能：
1. 读取智谱AI生成的JSON场景文件
2. 解析天气、时间、车辆数量等信息
3. 连接到CARLA服务器并自动配置场景
4. 生成对应数量的自动驾驶车辆

作者: AI Assistant
日期: 2026-04-13
"""

import carla
import json
import sys
import io
import os
import random
import time

# 设置控制台输出编码为UTF-8（Windows兼容）
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')


class CarlaSceneConfigurator:
    """CARLA场景配置器"""

    def __init__(self, host='localhost', port=2000, timeout=10.0):
        """初始化CARLA客户端"""
        print("=" * 70)
        print("CARLA AI场景配置器")
        print("=" * 70)

        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(timeout)
            print(f"\n[连接] 正在连接到 CARLA 服务器 ({host}:{port})...")
            self.world = self.client.get_world()
            print("✓ 连接成功")
            print(f"  当前地图: {self.world.get_map().name}")
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            raise

    def parse_json_file(self, json_path):
        """
        解析JSON场景文件

        Args:
            json_path: JSON文件路径

        Returns:
            dict: 解析后的场景配置
        """
        print(f"\n{'='*70}")
        print(f"[解析] 读取场景文件: {json_path}")
        print(f"{'='*70}")

        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # 提取 parsed_json 字段（如果存在）
            if 'parsed_json' in data:
                scene_data = data['parsed_json']
            else:
                scene_data = data

            # 智能提取字段（支持中英文键名）
            config = {
                'weather': self._extract_field(scene_data, ['weather', '天气']),
                'time': self._extract_field(scene_data, ['time', '时间']),
                'vehicle_count': self._extract_field(
                    scene_data,
                    ['vehicleCount', 'vehicle_count', '车辆数量']
                ),
                'location': self._extract_field(
                    scene_data,
                    ['location', '地点', '场景']
                ),
                'description': self._extract_field(
                    scene_data,
                    ['sceneDescription', '详细描述', 'description']
                )
            }

            # 转换车辆数量为整数
            if config['vehicle_count']:
                try:
                    config['vehicle_count'] = int(config['vehicle_count'])
                except (ValueError, TypeError):
                    config['vehicle_count'] = 5  # 默认值

            print(f"  天气: {config['weather']}")
            print(f"  时间: {config['time']}")
            print(f"  车辆数量: {config['vehicle_count']}")
            print(f"  地点: {config['location']}")

            return config

        except Exception as e:
            print(f"✗ 解析失败: {e}")
            raise

    def _extract_field(self, data, possible_keys):
        """从数据中提取字段，支持多个可能的键名"""
        for key in possible_keys:
            if key in data:
                return data[key]
        return None

    def configure_weather(self, weather_str, time_str=None):
        """
        根据天气和时间字符串配置CARLA天气

        Args:
            weather_str: 天气描述（如 "rainy", "下雨", "sunny" 等）
            time_str: 时间描述（如 "傍晚", "白天", "evening" 等）
        """
        print(f"\n{'='*70}")
        print(f"[天气] 配置天气场景")
        print(f"{'='*70}")

        # 创建基础天气参数
        weather = carla.WeatherParameters()

        # 根据天气类型设置参数
        weather_lower = str(weather_str).lower() if weather_str else ""

        if any(keyword in weather_lower for keyword in ['雨', 'rain', 'rainy']):
            # 雨天
            weather.cloudiness = 80.0
            weather.precipitation = 60.0
            weather.precipitation_deposits = 50.0
            weather.wind_intensity = 50.0
            weather.sun_azimuth_angle = -1.0
            print("  ✓ 设置为雨天模式")

        elif any(keyword in weather_lower for keyword in ['晴', 'sun', 'clear', 'sunny']):
            # 晴天
            weather.cloudiness = 10.0
            weather.precipitation = 0.0
            weather.precipitation_deposits = 0.0
            weather.wind_intensity = 10.0
            print("  ✓ 设置为晴天模式")

        elif any(keyword in weather_lower for keyword in ['阴', 'cloud', 'cloudy', 'overcast']):
            # 阴天
            weather.cloudiness = 90.0
            weather.precipitation = 0.0
            weather.precipitation_deposits = 0.0
            weather.wind_intensity = 20.0
            print("  ✓ 设置为阴天模式")

        elif any(keyword in weather_lower for keyword in ['雾', 'fog', 'foggy']):
            # 雾天
            weather.cloudiness = 70.0
            weather.precipitation = 0.0
            weather.fog_density = 50.0
            weather.fog_distance = 30.0
            print("  ✓ 设置为雾天模式")

        else:
            # 默认天气
            weather.cloudiness = 30.0
            weather.precipitation = 0.0
            print("  ✓ 使用默认天气")

        # 根据时间调整太阳高度角
        time_lower = str(time_str).lower() if time_str else ""

        if any(keyword in time_lower for keyword in ['傍晚', '黄昏', 'evening', 'dusk']):
            # 傍晚：太阳高度角较低
            weather.sun_altitude_angle = 10.0
            weather.sun_azimuth_angle = 200.0
            print("  ✓ 设置为傍晚时间（太阳高度角: 10°）")

        elif any(keyword in time_lower for keyword in ['早晨', '早上', 'morning', 'dawn']):
            # 早晨
            weather.sun_altitude_angle = 15.0
            weather.sun_azimuth_angle = 90.0
            print("  ✓ 设置为早晨时间（太阳高度角: 15°）")

        elif any(keyword in time_lower for keyword in ['中午', '正午', 'noon', 'midday']):
            # 中午
            weather.sun_altitude_angle = 70.0
            print("  ✓ 设置为中午时间（太阳高度角: 70°）")

        elif any(keyword in time_lower for keyword in ['夜晚', '晚上', 'night', 'midnight']):
            # 夜晚
            weather.sun_altitude_angle = -90.0
            print("  ✓ 设置为夜晚时间")

        else:
            # 默认：下午
            weather.sun_altitude_angle = 45.0
            print("  ✓ 使用默认时间（下午）")

        # 应用天气设置
        self.world.set_weather(weather)
        print(f"  ✓ 天气配置已应用")

    def spawn_vehicles(self, vehicle_count, autopilot=True):
        """
        在世界中生成指定数量的车辆

        Args:
            vehicle_count: 车辆数量
            autopilot: 是否启用自动驾驶

        Returns:
            list: 生成的车辆列表
        """
        print(f"\n{'='*70}")
        print(f"[车辆] 生成 {vehicle_count} 辆自动驾驶车辆")
        print(f"{'='*70}")

        # 获取所有可用的生成点
        spawn_points = self.world.get_map().get_spawn_points()

        if not spawn_points:
            print("  ✗ 未找到可用的生成点")
            return []

        # 随机选择生成点（避免重复）
        selected_spawns = random.sample(
            spawn_points,
            min(vehicle_count, len(spawn_points))
        )

        # 车辆蓝图库
        blueprint_library = self.world.get_blueprint_library()

        # 选择常见的车辆类型
        vehicle_blueprints = [
            bp for bp in blueprint_library.filter('vehicle.*')
            if bp.has_attribute('number_of_wheels') and
            int(bp.get_attribute('number_of_wheels').as_int()) == 4
        ]

        if not vehicle_blueprints:
            print("  ✗ 未找到可用的车辆蓝图")
            return []

        vehicles = []
        spawned_count = 0

        for i, spawn_point in enumerate(selected_spawns):
            try:
                # 随机选择车辆类型
                blueprint = random.choice(vehicle_blueprints)

                # 设置车辆颜色（随机）
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)

                # 尝试生成车辆
                vehicle = self.world.try_spawn_actor(blueprint, spawn_point)

                if vehicle is not None:
                    # 启用自动驾驶
                    if autopilot:
                        vehicle.set_autopilot(True)

                    vehicles.append(vehicle)
                    spawned_count += 1
                    print(f"  ✓ 车辆 {spawned_count}/{vehicle_count} 已生成 "
                          f"(类型: {blueprint.id.split('/')[-1]})")
                else:
                    print(f"  ⚠ 车辆 {i+1} 生成失败（位置可能被占用）")

            except Exception as e:
                print(f"  ✗ 车辆 {i+1} 生成错误: {e}")

        print(f"\n  总计成功生成: {spawned_count}/{vehicle_count} 辆车")
        return vehicles

    def cleanup_vehicles(self, vehicles):
        """清理生成的车辆"""
        print(f"\n[清理] 移除 {len(vehicles)} 辆车辆...")
        for vehicle in vehicles:
            try:
                vehicle.destroy()
            except:
                pass
        print("  ✓ 清理完成")

    def apply_scene(self, json_path):
        """
        应用完整的场景配置

        Args:
            json_path: JSON文件路径

        Returns:
            list: 生成的车辆列表
        """
        # 1. 解析JSON
        config = self.parse_json_file(json_path)

        # 2. 配置天气
        self.configure_weather(config['weather'], config['time'])

        # 3. 生成车辆
        vehicle_count = config['vehicle_count'] or 5
        vehicles = self.spawn_vehicles(vehicle_count)

        # 4. 输出总结
        print(f"\n{'='*70}")
        print("✅ 场景配置完成！")
        print(f"{'='*70}")
        print(f"  场景描述: {config.get('description', 'N/A')}")
        print(f"  天气: {config['weather']}")
        print(f"  时间: {config['time']}")
        print(f"  车辆数量: {len(vehicles)}")
        print(f"{'='*70}")

        return vehicles


def main():
    """主函数"""
    # 默认JSON文件路径
    default_json_dir = r"C:\Users\22317\OneDrive\Desktop\zhipu_test_results"
    default_json_file = os.path.join(default_json_dir, "result_01.json")

    # 从命令行参数获取文件路径，或使用默认值
    if len(sys.argv) > 1:
        json_file = sys.argv[1]
    else:
        json_file = default_json_file

    # 检查文件是否存在
    if not os.path.exists(json_file):
        print(f"✗ 文件不存在: {json_file}")
        print(f"\n提示: 请提供正确的JSON文件路径")
        print(f"用法: python carla_ai_scene.py [json_file_path]")
        sys.exit(1)

    print(f"\n目标JSON文件: {json_file}\n")

    try:
        # 创建场景配置器
        configurator = CarlaSceneConfigurator()

        # 应用场景配置
        vehicles = configurator.apply_scene(json_file)

        print("\n提示: 按 Ctrl+C 退出并清理车辆")

        # 保持运行，让用户观察场景
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n用户中断，正在清理...")
        if 'configurator' in locals():
            configurator.cleanup_vehicles(vehicles)
        print("再见！")

    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
