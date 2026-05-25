#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
螺丝壳山路 KML 路径自动驾驶演示脚本
=====================================
功能：
1. 解析 KML 文件中的路径点（手绘螺丝壳山路轨迹）
2. 将经纬度转换为 CARLA 世界坐标
3. 加载 Town05 地图并映射路径到道路表面
4. 实现纯跟踪控制器沿路径行驶
5. 生成动态交通流（10 辆车）
6. 设置浓雾天气
7. 实时 pygame 显示摄像头画面
8. 录制视频文件 kml_demo.mp4

运行方式：
    conda activate carla_0.9.13
    cd D:\Carla_0.9.13\PythonAPI\examples
    python run_kml_demo.py

输入文件：
    screw_shell.kml - 手绘路径 KML 文件（放在脚本同级目录）
"""

import os
import sys
import time
import csv
import math
import socket
import subprocess
import signal
from collections import deque
from xml.etree import ElementTree as ET

# ==================== KML 路径数据（硬编码备份） ====================
# 如果 KML 文件读取失败，使用这些硬编码的坐标点
# 数据来源：螺丝壳山路手绘轨迹，共 142 个路径点
KML_PATH_POINTS = [
    (107.43510331651247, 26.28062519089626),
    (107.4352168092053, 26.280574548499327),
    (107.43569505644946, 26.281192535561527),
    (107.43551670667031, 26.281279373899206),
    (107.43538108829377, 26.281470206302004),
    (107.43496401359415, 26.281213645308952),
    (107.43453027520466, 26.28090101868487),
    (107.43423094292142, 26.2806323923569),
    (107.43396826406456, 26.28044591289075),
    (107.43362616424754, 26.280390647895604),
    (107.43324742319236, 26.280231355164656),
    (107.4328564625903, 26.280219803832036),
    (107.43206847862054, 26.27979722565917),
    (107.4315492720487, 26.279659636452436),
    (107.43104230261739, 26.27939073794796),
    (107.43054755270283, 26.279269623138287),
    (107.4300833458339, 26.279285373097142),
    (107.42994284806919, 26.27955879445333),
    (107.43029098796173, 26.27974535428659),
    (107.43077963255236, 26.279718699213817),
    (107.43141487762331, 26.27985097115714),
    (107.43194016661613, 26.28036616136454),
    (107.43256931949206, 26.280810378692365),
    (107.43299080076801, 26.281117479464598),
    (107.4334367206857, 26.281512182101558),
    (107.43371770309737, 26.281977770313635),
    (107.43389483904886, 26.282388472011867),
    (107.43420026718067, 26.282799378099348),
    (107.4343835172779, 26.283193673150866),
    (107.43465230685416, 26.283440356420087),
    (107.4346950566998, 26.28368667513279),
    (107.43466450131015, 26.28383984734431),
    (107.43451176710457, 26.283905267144185),
    (107.4343468265526, 26.28379008581036),
    (107.43426131617102, 26.28349992173916),
    (107.43409027900559, 26.283209620673883),
    (107.43396200573096, 26.282930332384304),
    (107.4337176633029, 26.28263991596962),
    (107.43352830245718, 26.28237147660286),
    (107.43338169741678, 26.282234439750994),
    (107.43314919408513, 26.282033168981624),
    (107.43291057882747, 26.281984217936888),
    (107.43267197414613, 26.281804100189955),
    (107.43243337157803, 26.281633700731565),
    (107.43227068696925, 26.281560581151556),
    (107.4321080059545, 26.281448597569895),
    (107.43189109677856, 26.28135110778551),
    (107.43160369718086, 26.2811952169087),
    (107.43148983298363, 26.28096671431761),
    (107.43128376390112, 26.28106357187828),
    (107.43092045909118, 26.280858995086444),
    (107.43070897863899, 26.280863544786055),
    (107.4305517290595, 26.280805018706932),
    (107.4303348304969, 26.280799847559148),
    (107.43010709439741, 26.28071207409699),
    (107.42986851674029, 26.280614571016525),
    (107.42968416156856, 26.280565728440635),
    (107.42953234404251, 26.280473209364978),
    (107.4293263072361, 26.280356324518234),
    (107.42912568818613, 26.280346328953847),
    (107.42878410556546, 26.280204970076294),
    (107.42845879680561, 26.28002477248969),
    (107.42794932480759, 26.280293481686),
    (107.42793827972513, 26.280417094615927),
    (107.42800449973802, 26.280516085293478),
    (107.42812039092884, 26.28059536241956),
    (107.42841841358758, 26.280571039576497),
    (107.42886544870697, 26.280650772160676),
    (107.42925730677112, 26.28064636967354),
    (107.429433915806, 26.280735628006788),
    (107.42970987227108, 26.280844808006307),
    (107.4299637522739, 26.28099846538946),
    (107.43019555202275, 26.28124604990644),
    (107.43047151804804, 26.281365128359166),
    (107.43086339969372, 26.28147448879033),
    (107.43110624524543, 26.281781437662737),
    (107.43137670062993, 26.281949968191192),
    (107.43156988705901, 26.282039265819634),
    (107.43166922717441, 26.282311390219203),
    (107.43175752897007, 26.282568662770792),
    (107.43176302918869, 26.2828950406543),
    (107.43175749329316, 26.283162061303685),
    (107.43175747753565, 26.283424144571196),
    (107.43190649382994, 26.283755680134856),
    (107.43209415391907, 26.284032879710967),
    (107.43221006620128, 26.28414184369165),
    (107.43248053523503, 26.28432027265064),
    (107.43257988738907, 26.284463827195516),
    (107.43272892169999, 26.284592623600776),
    (107.43286691560874, 26.284741183311414),
    (107.4329276264366, 26.284919293309464),
    (107.43294417856058, 26.285057775738405),
    (107.43285585216272, 26.285126867600805),
    (107.43273440701495, 26.28517118437981),
    (107.43256880952522, 26.28509181211714),
    (107.43227350569262, 26.284809503006684),
    (107.4322326210057, 26.284500273414068),
    (107.43201466112507, 26.284298935078052),
    (107.43178388493784, 26.284045891033536),
    (107.43147616877539, 26.284016716125148),
    (107.43125180401493, 26.283849833041145),
    (107.43104666450833, 26.283832301563077),
    (107.4308928197498, 26.28367701147619),
    (107.43069409531284, 26.283642262945214),
    (107.43048897925792, 26.2833260923358),
    (107.43027744898323, 26.283107547297202),
    (107.43013643402827, 26.282894847603423),
    (107.42991209131723, 26.282601625002915),
    (107.42971338352828, 26.282446277001053),
    (107.42933520392064, 26.282135612152246),
    (107.42915573071751, 26.281997524704224),
    (107.42889933337031, 26.281962710625862),
    (107.42876701340647, 26.28191912392761),
    (107.42864340676257, 26.281944125052664),
    (107.42845800349167, 26.28189353150624),
    (107.428165850303, 26.281913272601834),
    (107.42818268996955, 26.282164999177787),
    (107.4283568462348, 26.282346460897795),
    (107.42859842419706, 26.282533049583353),
    (107.42878943082322, 26.282855490036987),
    (107.42907034495765, 26.283016966549866),
    (107.42930631394147, 26.283183417433985),
    (107.4295141890982, 26.28340520509135),
    (107.42957598055996, 26.283631822185882),
    (107.42983443202569, 26.28376810502514),
    (107.42995803158394, 26.28397970805881),
    (107.43013781349975, 26.284297104970424),
    (107.43016027843935, 26.28446829188135),
    (107.43010969890415, 26.284639374038136),
    (107.42992427668388, 26.284684415348202),
    (107.42985684576895, 26.28478499854662),
    (107.42922476128089, 26.284570167619666),
    (107.4290927987954, 26.284542663893284),
    (107.42898478415106, 26.284424223833604),
    (107.42877474508114, 26.28439705036357),
    (107.42857670771528, 26.28441290992847),
    (107.42831267348346, 26.284224361353555),
    (107.42803063440668, 26.284127197978815),
    (107.42772459429237, 26.284056890366852),
    (107.42750856932862, 26.283997458969875),
    (107.42732254854305, 26.283964953204432),
    (107.42694451147652, 26.2838999383329),
]

# ==================== 日志工具 ====================
def log_info(msg):
    """打印信息日志"""
    print(f"[INFO] {time.strftime('%H:%M:%S')} - {msg}")

def log_error(msg):
    """打印错误日志"""
    print(f"[ERROR] {time.strftime('%H:%M:%S')} - {msg}")

def log_success(msg):
    """打印成功日志"""
    print(f"[SUCCESS] {time.strftime('%H:%M:%S')} - {msg}")

def log_warning(msg):
    """打印警告日志"""
    print(f"[WARNING] {time.strftime('%H:%M:%S')} - {msg}")

# ==================== 环境准备 ====================
def check_and_install_dependencies():
    """检查并安装必要的 Python 依赖包"""
    log_info("步骤 1/10: 检查 Python 依赖...")
    
    required_packages = {
        'pygame': 'pygame',
        'cv2': 'opencv-python',
        'numpy': 'numpy',
        'pyproj': 'pyproj',
        'scipy': 'scipy',
    }
    
    missing_packages = []
    for package_name, pip_name in required_packages.items():
        try:
            __import__(package_name)
            log_info(f"  ✓ {package_name} 已安装")
        except ImportError:
            log_warning(f"  ✗ {package_name} 未安装，正在安装...")
            missing_packages.append(pip_name)
    
    if missing_packages:
        log_info("正在使用 pip 安装缺失的包...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install"] + missing_packages)
            log_success("所有依赖包安装完成")
        except subprocess.CalledProcessError as e:
            log_error(f"安装包失败：{e}")
            log_warning("请手动运行：pip install " + " ".join(missing_packages))
            return False
    
    # 特别检查 carla 模块
    try:
        import carla
        log_success("CARLA 模块已就绪")
        return True
    except ImportError:
        log_error("未找到 carla 模块！")
        log_info("请确保已激活 conda 环境：conda activate carla_0.9.13")
        return False

# ==================== 解析 KML 文件 ====================
def parse_kml_file(kml_path):
    """解析 KML 文件，提取路径坐标"""
    log_info(f"解析 KML 文件：{kml_path}")
    
    try:
        if not os.path.exists(kml_path):
            log_warning(f"KML 文件不存在：{kml_path}，使用硬编码坐标")
            return KML_PATH_POINTS
        
        # 尝试解析 KML 文件
        tree = ET.parse(kml_path)
        root = tree.getroot()
        
        # 命名空间
        ns = {
            'kml': 'http://www.opengis.net/kml/2.2',
            'gx': 'http://www.google.com/kml/ext/2.2'
        }
        
        # 查找 LineString 中的 coordinates
        coordinates_elem = root.find('.//kml:coordinates', ns)
        
        if coordinates_elem is None:
            # 尝试不带命名空间的查找
            coordinates_elem = root.find('.//coordinates')
        
        if coordinates_elem is None:
            log_warning("未找到 coordinates 元素，使用硬编码坐标")
            return KML_PATH_POINTS
        
        # 解析坐标字符串
        coord_text = coordinates_elem.text.strip()
        coord_list = []
        
        for coord_str in coord_text.split():
            parts = coord_str.split(',')
            if len(parts) >= 2:
                try:
                    lon = float(parts[0])
                    lat = float(parts[1])
                    if -180 <= lon <= 180 and -90 <= lat <= 90:
                        coord_list.append((lon, lat))
                except ValueError:
                    continue
        
        if len(coord_list) > 0:
            log_success(f"成功从 KML 文件解析 {len(coord_list)} 个路径点")
            return coord_list
        else:
            log_warning("KML 文件中未找到有效坐标，使用硬编码坐标")
            return KML_PATH_POINTS
    
    except Exception as e:
        log_warning(f"解析 KML 失败：{e}，使用硬编码坐标")
        return KML_PATH_POINTS

# ==================== 坐标转换 ====================
class GPSConverter:
    """GPS 坐标与 CARLA 世界坐标转换器"""
    
    def __init__(self):
        try:
            from pyproj import Transformer
            # UTM 48N (螺丝壳位于此区域)
            self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:32648", always_xy=True)
            log_info("使用 UTM 48N 投影 (EPSG:32648)")
        except Exception as e:
            log_warning(f"pyproj 初始化失败：{e}，使用简单投影")
            self.transformer = None
            self.lat0 = 26.28  # 参考纬度
            self.lon0 = 107.43  # 参考经度
            self.R = 6378137.0  # 地球半径
    
    def gps_to_xy(self, lon, lat):
        """将 GPS 坐标转换为平面坐标 (x, y)"""
        if self.transformer:
            x, y = self.transformer.transform(lon, lat)
            return x, y
        else:
            # 简单墨卡托投影
            dlat = lat - self.lat0
            dlon = lon - self.lon0
            dx = dlon * math.pi * self.R / 180.0 * math.cos(math.radians(self.lat0))
            dy = dlat * math.pi * self.R / 180.0
            return dx, dy
    
    def gps_to_carla(self, lon, lat, alt=0.0):
        """将 GPS 坐标转换为 CARLA 世界坐标"""
        x, y = self.gps_to_xy(lon, lat)
        # CARLA 坐标系：X 前，Y 右，Z 上
        return (x, -y, alt)

# ==================== 路径平滑与曲率计算 ====================
def smooth_path(path_points, interval=2.0):
    """使用样条插值平滑路径"""
    try:
        from scipy.interpolate import splprep, splev
        
        if len(path_points) < 3:
            return path_points
        
        # 转换为数组
        import numpy as np
        points = np.array(path_points)
        
        # 样条插值
        tck, u = splprep([points[:, 0], points[:, 1], points[:, 2]], s=0.5, k=3)
        
        # 计算总长度
        u_fine = np.linspace(0, 1, 1000)
        fine_points = np.array(splev(u_fine, tck))
        
        # 按固定间隔采样
        smoothed = []
        total_len = 0
        prev_point = fine_points[:, 0]
        
        for i in range(1, len(u_fine)):
            curr_point = fine_points[:, i]
            seg_len = np.linalg.norm(curr_point - prev_point)
            total_len += seg_len
            
            if total_len >= interval:
                smoothed.append((curr_point[0], curr_point[1], curr_point[2]))
                total_len = 0
            
            prev_point = curr_point
        
        log_info(f"路径平滑完成，原始点数：{len(path_points)}, 平滑后点数：{len(smoothed)}")
        return smoothed
    
    except Exception as e:
        log_warning(f"路径平滑失败：{e}，使用原始路径")
        return path_points

def calculate_curvature(p1, p2, p3):
    """计算三点之间的曲率"""
    import numpy as np
    
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p3) - np.array(p2)
    
    # 计算夹角
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)
    
    if v1_norm < 0.1 or v2_norm < 0.1:
        return 0.0
    
    cos_angle = np.dot(v1, v2) / (v1_norm * v2_norm)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = math.acos(cos_angle)
    
    # 曲率 = 角度 / 弧长
    arc_len = (v1_norm + v2_norm) / 2.0
    if arc_len < 0.1:
        return 0.0
    
    curvature = angle / arc_len
    return curvature

# ==================== 纯跟踪控制器 ====================
class PurePursuitController:
    """纯跟踪路径跟踪控制器"""
    
    def __init__(self, vehicle, path_points, lookahead_distance=5.0, speed_limit=30.0):
        """
        初始化纯跟踪控制器
        
        参数:
            vehicle: CARLA 车辆对象
            path_points: 路径点列表 [(x1, y1, z1), (x2, y2, z2), ...]
            lookahead_distance: 前视距离（米）
            speed_limit: 速度限制（km/h）
        """
        import carla
        self.vehicle = vehicle
        self.path_points = path_points
        self.base_lookahead_distance = lookahead_distance
        self.speed_limit = speed_limit / 3.6  # 转换为 m/s
        
        self.current_point_index = 0
        self.total_points = len(path_points)
        
        # 预计算曲率和目标速度
        self.curvatures = []
        self.target_speeds = []
        for i in range(len(path_points) - 2):
            curvature = calculate_curvature(path_points[i], path_points[i+1], path_points[i+2])
            self.curvatures.append(curvature)
            # 曲率大时降低速度
            if curvature > 0.1:
                self.target_speeds.append(20.0 / 3.6)
            else:
                self.target_speeds.append(30.0 / 3.6)
        
        # 填充最后两个点
        if len(self.curvatures) > 0:
            self.curvatures.append(self.curvatures[-1])
            self.target_speeds.append(self.target_speeds[-1])
        else:
            self.curvatures = [0.0] * len(path_points)
            self.target_speeds = [self.speed_limit] * len(path_points)
    
    def get_current_location(self):
        """获取车辆当前位置"""
        transform = self.vehicle.get_transform()
        return transform.location
    
    def get_target_point(self):
        """根据前视距离找到目标点"""
        current_loc = self.get_current_location()
        
        # 动态调整前视距离
        current_speed = self.vehicle.get_velocity()
        speed_magnitude = math.sqrt(current_speed.x**2 + current_speed.y**2 + current_speed.z**2)
        lookahead = self.base_lookahead_distance + 0.5 * speed_magnitude
        
        # 从当前点索引开始搜索
        for i in range(self.current_point_index, self.total_points):
            target_loc = carla.Location(*self.path_points[i])
            
            # 计算到目标点的距离
            distance = math.sqrt(
                (current_loc.x - target_loc.x)**2 +
                (current_loc.y - target_loc.y)**2
            )
            
            # 找到第一个满足前视距离的点
            if distance >= lookahead:
                self.current_point_index = i
                return target_loc
        
        # 如果没有找到，返回最后一个点
        return carla.Location(*self.path_points[-1])
    
    def calculate_steering(self, target_point):
        """计算转向角度"""
        current_loc = self.get_current_location()
        current_transform = self.vehicle.get_transform()
        
        # 计算到目标点的向量
        to_target = target_point - current_loc
        
        # 计算车辆前进方向向量
        forward_vector = current_transform.get_forward_vector()
        
        # 计算两个向量的夹角
        dot_product = to_target.x * forward_vector.x + to_target.y * forward_vector.y
        magnitude_to_target = math.sqrt(to_target.x**2 + to_target.y**2)
        magnitude_forward = math.sqrt(forward_vector.x**2 + forward_vector.y**2)
        
        if magnitude_to_target < 0.1 or magnitude_forward < 0.1:
            return 0.0
        
        cos_angle = dot_product / (magnitude_to_target * magnitude_forward)
        cos_angle = max(-1.0, min(1.0, cos_angle))  # 限制在 [-1, 1]
        angle = math.acos(cos_angle)
        
        # 判断转向方向（左转为正，右转为负）
        cross_product = to_target.x * forward_vector.y - to_target.y * forward_vector.x
        if cross_product < 0:
            angle = -angle
        
        return angle
    
    def run_step(self):
        """执行一步控制"""
        import carla
        
        # 获取目标点
        target_point = self.get_target_point()
        
        # 计算转向角度
        steer_angle = self.calculate_steering(target_point)
        
        # 归一化到 [-1, 1]
        steer_angle = max(-1.0, min(1.0, steer_angle / math.pi))
        
        # 获取当前速度
        current_speed = self.vehicle.get_velocity()
        speed_magnitude = math.sqrt(current_speed.x**2 + current_speed.y**2 + current_speed.z**2)
        
        # 根据曲率确定目标速度
        if self.current_point_index < len(self.target_speeds):
            target_speed = self.target_speeds[self.current_point_index]
        else:
            target_speed = self.speed_limit
        
        # 简单的 P 控制
        speed_error = target_speed - speed_magnitude
        throttle = max(0.0, min(1.0, 0.5 + speed_error * 0.2))
        brake = 0.0
        
        # 如果速度超过目标速度，刹车
        if speed_magnitude > target_speed * 1.1:
            throttle = 0.0
            brake = 0.5
        
        # 创建控制指令
        control = carla.VehicleControl(
            throttle=throttle,
            steer=steer_angle,
            brake=brake,
            reverse=False,
            manual_gear_shift=False
        )
        
        self.vehicle.apply_control(control)
        
        return throttle, steer_angle, speed_magnitude * 3.6  # 返回 km/h
    
    def get_progress(self):
        """获取路径进度"""
        if self.total_points == 0:
            return 0.0
        return (self.current_point_index / self.total_points) * 100.0


# ==================== 启动 CARLA 服务器 ====================
def is_port_available(port):
    """检查端口是否可用"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(('localhost', port))
            return True
        except socket.error:
            return False

def start_carla_server(carla_path, timeout=90):
    """启动 CARLA 服务器"""
    log_info("步骤 2/10: 启动 CARLA 服务器...")
    
    if not os.path.exists(carla_path):
        log_error(f"CARLA 可执行文件不存在：{carla_path}")
        return None
    
    # 检查端口 2000 是否已被占用
    if not is_port_available(2000):
        log_info("检测到 CARLA 服务器已在运行（端口 2000 被占用）")
        log_info("将使用现有的 CARLA 服务器")
        time.sleep(2)
        return True
    
    # 启动 CARLA - 使用极低画质以节省显存
    log_info(f"启动 CARLA: {carla_path}")
    log_info("参数：-quality-level=Low -ResX=320 -ResY=240 -windowed -opengl3 (极低画质以节省显存)")
    
    cmd = [
        carla_path,
        "-quality-level=Low",
        "-ResX=320",
        "-ResY=240",
        "-windowed",
        "-opengl3",  # 使用 OpenGL 3 节省显存
        "-benchmark",  # 基准模式，优化性能
        "-FPS=20"  # 限制帧率
    ]
    
    try:
        # 在后台启动 CARLA
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        log_info("CARLA 服务器启动中...")
        
        # 等待端口 2000 可用，并且额外等待确保 CARLA 完全就绪
        start_time = time.time()
        while time.time() - start_time < timeout:
            if is_port_available(2000):
                log_success("CARLA 服务器已就绪（端口 2000 开放）")
                # 额外等待 10 秒确保 CARLA 完全初始化
                log_info("等待 CARLA 完全初始化...")
                time.sleep(10)
                return process
            
            # 检查进程是否还在运行
            if process.poll() is not None:
                log_error("CARLA 服务器进程意外退出")
                return None
            
            time.sleep(2)
            elapsed = int(time.time() - start_time)
            log_info(f"  等待服务器启动... ({elapsed}/{timeout}秒)")
        
        log_error(f"等待 CARLA 服务器超时（{timeout}秒）")
        return None
        
    except Exception as e:
        log_error(f"启动 CARLA 失败：{e}")
        return None


# ==================== 加载地图 ====================
def load_builtin_map(client, map_name="Town05"):
    """加载 CARLA 内置地图"""
    log_info(f"加载内置地图：{map_name}...")
    
    try:
        # 增加超时时间（CARLA 0.9.13 兼容性）
        client.set_timeout(120.0)  # 增加到 120 秒
        
        available_maps = client.get_available_maps()
        
        # 在 CARLA 0.9.13 中，地图名称可能是完整路径
        # 尝试直接匹配或查找包含地图名称的路径
        target_map = None
        if map_name in available_maps:
            target_map = map_name
        else:
            # 查找包含地图名称的路径（例如 '/Game/Carla/Maps/Town05'）
            for map_path in available_maps:
                if map_name in map_path and not map_path.endswith('_Opt'):
                    target_map = map_path
                    break
        
        if not target_map:
            log_error(f"地图 {map_name} 不可用，可用地图：{available_maps}")
            return False
        
        log_info(f"正在加载 {target_map}，这可能需要 1-2 分钟...")
        client.load_world(target_map)
        log_success(f"内置地图 {map_name} 加载成功")
        return True
        
    except Exception as e:
        log_error(f"加载内置地图失败：{e}")
        return False


# ==================== 天气设置 ====================
def set_foggy_weather(world):
    """设置浓雾天气"""
    log_info("步骤 5/10: 设置浓雾天气...")
    
    try:
        import carla
        
        weather = carla.WeatherParameters(
            fog_density=95.0,      # 雾浓度 95%
            fog_distance=15.0,     # 雾距离 15 米
            fog_falloff=0.5,       # 雾衰减
            clouds_density=80.0,   # 云浓度
            clouds_opacity=0.9,    # 云不透明度
            precipitation=0.0,     # 无降水
            sun_altitude_angle=30.0,  # 太阳高度角
            sun_azimuth_angle=180.0,  # 太阳方位角
        )
        
        world.set_weather(weather)
        log_success("浓雾天气设置完成（能见度<20 米）")
        
    except Exception as e:
        log_error(f"设置天气失败：{e}")


# ==================== 生成交通流 ====================
def generate_traffic(world, num_vehicles=5):
    """生成动态交通流"""
    log_info(f"步骤 6/10: 生成 {num_vehicles} 辆动态车辆...")
    
    try:
        blueprint_library = world.get_blueprint_library()
        spawn_points = world.get_map().get_spawn_points()
        
        if len(spawn_points) < num_vehicles:
            log_warning(f"可用出生点不足 ({len(spawn_points)})，将生成尽可能多的车辆")
            num_vehicles = len(spawn_points)
        
        vehicles = []
        for i in range(num_vehicles):
            try:
                # 随机选择车辆模型
                vehicle_bp = blueprint_library.filter('vehicle.*')[0]
                spawn_point = spawn_points[i % len(spawn_points)]
                
                vehicle = world.spawn_actor(vehicle_bp, spawn_point)
                vehicles.append(vehicle)
                log_info(f"  生成车辆 {i+1}/{num_vehicles}")
                
            except Exception as e:
                log_warning(f"  生成车辆 {i+1} 失败：{e}")
                continue
        
        log_success(f"成功生成 {len(vehicles)} 辆动态车辆")
        return vehicles
        
    except Exception as e:
        log_error(f"生成交通流失败：{e}")
        return []


# ==================== 路径映射到道路 ====================
def map_path_to_road(world, path_points_xy):
    """将路径点映射到道路表面"""
    log_info("步骤 4/10: 将路径点映射到道路表面...")
    
    try:
        import carla
        carla_map = world.get_map()
        
        mapped_points = []
        valid_count = 0
        invalid_count = 0
        
        for i, (x, y) in enumerate(path_points_xy):
            # 创建初始位置（Z=0）
            location = carla.Location(x, y, 0)
            
            # 获取最近的道路点
            waypoint = carla_map.get_waypoint(location, project_to_road=True)
            
            if waypoint:
                # 检查距离
                distance = math.sqrt(
                    (waypoint.transform.location.x - x)**2 +
                    (waypoint.transform.location.y - y)**2
                )
                
                if distance < 5.0:  # 5 米阈值
                    # 使用道路点的真实位置（包含高度）
                    mapped_points.append((
                        waypoint.transform.location.x,
                        waypoint.transform.location.y,
                        waypoint.transform.location.z
                    ))
                    valid_count += 1
                else:
                    log_warning(f"  点 {i} 距离道路过远 ({distance:.2f}米)，丢弃")
                    invalid_count += 1
            else:
                log_warning(f"  点 {i} 未找到道路点，丢弃")
                invalid_count += 1
        
        log_success(f"路径映射完成：有效点 {valid_count} 个，无效点 {invalid_count} 个")
        
        if valid_count < 10:
            log_error("有效路径点过少，路径映射失败")
            return None
        
        return mapped_points
        
    except Exception as e:
        log_error(f"路径映射失败：{e}")
        return None


# ==================== Pygame 可视化与视频录制 ====================
class DisplayAndRecorder:
    """显示和视频录制管理器"""
    
    def __init__(self, width=1280, height=720, output_file="kml_demo.mp4", fps=20):
        """初始化显示和录制"""
        import pygame
        import cv2
        import numpy as np
        
        self.width = width
        self.height = height
        self.fps = fps
        self.output_file = output_file
        
        # 初始化 pygame
        pygame.init()
        self.display = pygame.display.set_mode((width, height))
        pygame.display.set_caption("螺丝壳山路 KML 路径自动驾驶演示")
        self.font = pygame.font.Font(None, 28)
        self.clock = pygame.time.Clock()
        
        # 初始化视频录制
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
        
        self.frame_count = 0
        self.recording = True
        
        log_info(f"初始化显示窗口：{width}x{height}")
        log_info(f"视频录制：{output_file}, {fps}fps")
    
    def update(self, image_array, speed, progress, waypoint_count):
        """更新显示和录制"""
        import pygame
        import cv2
        import numpy as np
        
        # 处理事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
        
        # 转换图像为 pygame surface
        # image_array 应该是 (H, W, 3) 的 RGB 数组
        if image_array is not None:
            surface = pygame.surfarray.make_surface(image_array.swapaxes(0, 1))
            surface = pygame.transform.scale(surface, (self.width, self.height))
            self.display.blit(surface, (0, 0))
        
        # 绘制叠加信息
        self.draw_overlay(speed, progress, waypoint_count)
        
        # 更新显示
        pygame.display.flip()
        
        # 录制视频
        if self.recording:
            # 从 pygame 表面获取像素数据
            frame_data = pygame.image.tostring(self.display, 'RGB')
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((self.height, self.width, 3))
            # RGB 转 BGR（OpenCV 使用 BGR）
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.video_writer.write(frame)
            self.frame_count += 1
        
        self.clock.tick(self.fps)
        return True
    
    def draw_overlay(self, speed, progress, waypoint_count):
        """在画面上叠加信息"""
        import pygame
        
        # 创建信息面板
        pygame.draw.rect(self.display, (0, 0, 0, 180), (10, 10, 300, 120))
        pygame.draw.rect(self.display, (255, 255, 255, 180), (10, 10, 300, 120), 2)
        
        # 渲染文本
        speed_text = self.font.render(f"速度：{speed:.1f} km/h", True, (255, 255, 255))
        progress_text = self.font.render(f"进度：{progress:.1f}%", True, (255, 255, 255))
        waypoint_text = self.font.render(f"路径点：{waypoint_count}", True, (255, 255, 255))
        weather_text = self.font.render(f"天气：浓雾", True, (255, 255, 255))
        
        self.display.blit(speed_text, (20, 20))
        self.display.blit(progress_text, (20, 50))
        self.display.blit(waypoint_text, (20, 80))
        self.display.blit(weather_text, (20, 110))
    
    def close(self):
        """关闭并保存视频"""
        import pygame
        
        log_info(f"录制帧数：{self.frame_count}")
        
        # 停止录制
        self.recording = False
        
        # 关闭视频写入器
        self.video_writer.release()
        log_success(f"视频已保存到：{self.output_file}")
        
        # 关闭 pygame
        pygame.quit()


# ==================== 主函数 ====================
def main():
    """主函数"""
    import carla
    import pygame
    import numpy as np
    
    print("=" * 70)
    print("螺丝壳山路 KML 路径自动驾驶演示")
    print("=" * 70)
    
    # 配置参数
    CARLA_PATH = r"D:\Carla_0.9.13\CarlaUE4.exe"
    WORK_DIR = r"D:\Carla_0.9.13\PythonAPI\examples"
    KML_FILE = os.path.join(WORK_DIR, "screw_shell.kml")
    
    carla_process = None
    
    try:
        # 步骤 1: 检查和安装依赖
        if not check_and_install_dependencies():
            log_error("依赖检查失败，程序退出")
            return 1
        
        # 步骤 2: 启动 CARLA 服务器
        carla_process = start_carla_server(CARLA_PATH)
        if carla_process is None:
            log_error("CARLA 服务器启动失败，程序退出")
            return 1
        
        # 连接到 CARLA
        log_info("连接到 CARLA 服务器...")
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)  # 增加到 30 秒
        log_success("已连接到 CARLA 服务器")
        
        # 步骤 3: 解析 KML 文件并转换坐标
        log_info("\n步骤 3/10: 解析 KML 路径文件...")
        kml_points = parse_kml_file(KML_FILE)
        
        # 转换为平面坐标
        converter = GPSConverter()
        path_points_xy = []
        for lon, lat in kml_points:
            x, y = converter.gps_to_xy(lon, lat)
            path_points_xy.append((x, y))
        
        log_success(f"坐标转换完成，共 {len(path_points_xy)} 个点")
        
        # 步骤 4: 加载地图并映射路径
        if not load_builtin_map(client, "Town05"):
            log_error("加载地图失败，程序退出")
            return 1
        
        # 获取世界对象
        world = client.get_world()
        
        # 映射路径到道路
        mapped_path = map_path_to_road(world, path_points_xy)
        
        if mapped_path is None:
            log_warning("路径映射失败，回退到自动驾驶模式")
            use_pure_pursuit = False
        else:
            # 路径平滑
            smoothed_path = smooth_path(mapped_path, interval=2.0)
            use_pure_pursuit = True
        
        # 步骤 5: 设置天气
        set_foggy_weather(world)
        
        # 步骤 6: 生成交通流
        traffic_vehicles = generate_traffic(world, num_vehicles=5)
        
        # 步骤 7: 生成自车
        log_info("\n步骤 7/10: 生成自车...")
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        
        # 选择起点
        if use_pure_pursuit and len(smoothed_path) > 0:
            # 使用路径的第一个点作为起点
            start_location = carla.Location(*smoothed_path[0])
            start_rotation = carla.Rotation()
            spawn_transform = carla.Transform(start_location, start_rotation)
        else:
            # 使用地图默认出生点
            spawn_points = world.get_map().get_spawn_points()
            spawn_transform = spawn_points[0] if spawn_points else carla.Transform()
        
        ego_vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
        log_success("自车生成成功")
        
        # 设置控制器
        if use_pure_pursuit:
            log_info("使用纯跟踪控制器沿路径行驶")
            controller = PurePursuitController(ego_vehicle, smoothed_path, lookahead_distance=5.0, speed_limit=30.0)
        else:
            log_info("启用 CARLA 自动驾驶模式（备选方案）")
            ego_vehicle.set_autopilot(True)
        
        # 步骤 8: 安装摄像头
        log_info("\n步骤 8/10: 安装摄像头传感器...")
        
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1280')
        camera_bp.set_attribute('image_size_y', '720')
        camera_bp.set_attribute('fov', '90')
        
        # 摄像头安装在车顶前方
        camera_transform = carla.Transform(
            carla.Location(x=1.5, z=2.0),
            carla.Rotation(pitch=-10)
        )
        
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        
        # 图像队列
        image_queue = deque(maxlen=1)
        
        def on_image(image):
            """摄像头回调函数"""
            # 从 CARLA 图像转换为 numpy 数组
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]  # 去掉 alpha 通道
            array = array[:, :, ::-1]  # BGR 转 RGB
            image_queue.append(array)
        
        camera.listen(on_image)
        log_success("摄像头安装完成")
        
        # 步骤 9: 初始化显示和录制
        log_info("\n步骤 9/10: 初始化显示和视频录制...")
        display_recorder = DisplayAndRecorder(width=1280, height=720, output_file="kml_demo.mp4", fps=20)
        log_success("所有系统就绪！按 ESC 退出")
        log_info("=" * 70)
        
        # 步骤 10: 主循环
        log_info("\n开始自动驾驶演示...")
        
        running = True
        frame_count = 0
        
        while running:
            # 更新控制器
            if use_pure_pursuit:
                try:
                    throttle, steer, speed = controller.run_step()
                    progress = controller.get_progress()
                    waypoint_count = controller.current_point_index
                except Exception as e:
                    log_warning(f"控制器执行错误：{e}")
                    throttle, steer, speed, progress, waypoint_count = 0, 0, 0, 0, 0
            else:
                speed = 0.0
                progress = 0.0
                waypoint_count = 0
            
            # 处理图像
            image_array = None
            if image_queue:
                image_array = image_queue[0]
            
            # 更新显示和录制
            running = display_recorder.update(image_array, speed, progress, waypoint_count)
            
            frame_count += 1
            if frame_count % 60 == 0:  # 每 60 帧打印一次状态
                log_info(f"运行中... 速度：{speed:.1f} km/h, 进度：{progress:.1f}%, 帧数：{frame_count}")
        
        log_info("退出主循环...")
        
    except KeyboardInterrupt:
        log_info("用户中断")
    
    except Exception as e:
        log_error(f"程序异常：{e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 清理资源
        log_info("清理资源...")
        
        try:
            if 'display_recorder' in locals():
                display_recorder.close()
            
            if 'camera' in locals():
                camera.stop()
                camera.destroy()
            
            if 'ego_vehicle' in locals():
                ego_vehicle.destroy()
            
            if 'traffic_vehicles' in locals():
                for v in traffic_vehicles:
                    v.destroy()
            
        except Exception as e:
            log_warning(f"清理资源时出错：{e}")
        
        # 关闭 CARLA 进程
        if carla_process:
            log_info("关闭 CARLA 服务器...")
            carla_process.terminate()
            try:
                carla_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                carla_process.kill()
    
    log_success("演示结束，感谢使用！")
    return 0

if __name__ == "__main__":
    sys.exit(main())
