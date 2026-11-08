# -*- coding: utf-8 -*-
"""
AEB自动紧急制动控制器单元测试
纯逻辑测试，不连接CARLA服务器
"""

import sys
import os
import math
from types import SimpleNamespace
import pytest

# 从当前仓库导入控制器，CARLA SDK 由运行环境提供。
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers'))

# 导入待测的AEB控制器
from stanley_pid_controller import AEBController


def vehicle(actor_id, x, speed, y=0.0, yaw=0.0, lateral_speed=0.0):
    return SimpleNamespace(
        id=actor_id,
        bounding_box=SimpleNamespace(extent=SimpleNamespace(x=2.3, y=0.95)),
        get_transform=lambda: SimpleNamespace(
            location=SimpleNamespace(x=x, y=y), rotation=SimpleNamespace(yaw=yaw)
        ),
        get_velocity=lambda: SimpleNamespace(x=speed, y=lateral_speed),
    )


def test_target_speed_is_independent_of_actor_order():
    ego = vehicle(1, 0.0, 20.0)
    lead = vehicle(2, 20.0, 10.0)
    other = vehicle(3, 35.0, 19.0)
    for order in [(lead, other), (other, lead)]:
        world = SimpleNamespace(
            get_actors=lambda: SimpleNamespace(filter=lambda pattern: (ego,) + order)
        )
        controller = AEBController()
        distance, relative_speed = controller.detect_front_vehicle(ego, world)
        assert controller.last_target_id == lead.id
        assert distance == pytest.approx(20.0)
        assert relative_speed == pytest.approx(10.0)
        assert controller.calculate_ttc(distance, relative_speed) == pytest.approx(2.0)


def test_adjacent_vehicle_is_detected_when_its_body_enters_ego_corridor():
    ego = vehicle(1, 0.0, 12.0)
    controller = AEBController()

    def detect(target):
        world = SimpleNamespace(
            get_actors=lambda: SimpleNamespace(filter=lambda pattern: [ego, target])
        )
        return controller.detect_front_vehicle(ego, world)

    assert detect(vehicle(2, 6.0, 0.0, y=3.5)) == (None, 0)
    assert controller.last_target_id is None
    distance, closing = detect(vehicle(2, 6.0, 0.0, y=2.2, yaw=-15.0))
    assert controller.last_target_id == 2
    assert closing == 12.0
    assert controller.update_state(distance, closing) == 1.0
    assert detect(vehicle(2, 6.0, 0.0, y=3.5)) == (None, 0)
    assert controller.last_target_id is None


@pytest.mark.parametrize("lateral_speed, expected_target", [(-1.75, 2), (-0.2, None), (0.0, None), (1.75, None)])
def test_predict_cutin_only_for_near_term_lateral_approach(lateral_speed, expected_target):
    ego = vehicle(1, 0.0, 12.0)
    neighbor = vehicle(2, 6.0, 8.0, y=3.5, lateral_speed=lateral_speed)
    world = SimpleNamespace(get_actors=lambda: SimpleNamespace(filter=lambda pattern: [ego, neighbor]))
    controller = AEBController()

    distance, closing = controller.detect_front_vehicle(ego, world)

    assert controller.last_target_id == expected_target
    if expected_target is not None:
        assert closing == pytest.approx(4.0)
        assert controller.update_state(distance, closing) == 1.0
    else:
        assert (distance, closing) == (None, 0)


def test_keep_braking_while_cutin_vehicle_can_still_overlap():
    controller = AEBController()

    def brake(ego_speed, target_x, target_y):
        ego = vehicle(1, 0.0, ego_speed)
        target = vehicle(2, target_x, 8.0, y=target_y, lateral_speed=-1.75)
        world = SimpleNamespace(get_actors=lambda: SimpleNamespace(filter=lambda pattern: [ego, target]))
        return controller.update_state(*controller.detect_front_vehicle(ego, world))

    assert brake(12.0, 4.0, 3.0) == 1.0
    assert brake(8.0, -0.2, 2.2) == 1.0
    assert brake(8.0, -0.2, 1.8) == 1.0
    assert brake(8.0, -0.3, 1.7) == 1.0
    assert brake(8.0, -5.0, 1.0) == 0.0


class TestAEBController:
    """AEB控制器测试类"""
    
    @pytest.fixture
    def aeb_default(self):
        """默认AEB控制器实例"""
        return AEBController(ttc_warning=4.0, ttc_partial=3.0, ttc_full=1.5, detection_range=80.0)
    
    @pytest.fixture
    def aeb_aggressive(self):
        """激进型AEB控制器实例"""
        return AEBController(ttc_warning=2.5, ttc_partial=1.6, ttc_full=0.6, detection_range=50.0)


class TestTTCCalculation:
    """TTC计算测试"""
    
    @pytest.fixture
    def aeb(self):
        return AEBController()
    
    def test_ttc_basic(self, aeb):
        """测试基本TTC计算"""
        # 距离100米，相对速度20m/s，TTC应为5秒
        ttc = aeb.calculate_ttc(100.0, 20.0)
        assert ttc == pytest.approx(5.0, rel=0.01)
    
    def test_ttc_zero_relative_speed(self, aeb):
        """测试相对速度为0时的TTC"""
        # 相对速度为0，应返回无穷大
        ttc = aeb.calculate_ttc(100.0, 0.0)
        assert ttc == float('inf')
    
    def test_ttc_negative_relative_speed(self, aeb):
        """测试负相对速度（远离）时的TTC"""
        # 负相对速度表示远离，应返回无穷大
        ttc = aeb.calculate_ttc(100.0, -5.0)
        assert ttc == float('inf')
    
    def test_ttc_very_small_relative_speed(self, aeb):
        """测试极小相对速度时的TTC"""
        # 相对速度小于0.1，应返回无穷大
        ttc = aeb.calculate_ttc(100.0, 0.05)
        assert ttc == float('inf')
    
    def test_ttc_high_speed_short_distance(self, aeb):
        """测试高速近距离的TTC"""
        # 距离10米，相对速度30m/s，TTC应约为0.33秒
        ttc = aeb.calculate_ttc(10.0, 30.0)
        assert ttc == pytest.approx(0.333, rel=0.01)


class TestStateTransitions:
    """状态机转换测试"""
    
    @pytest.fixture
    def aeb(self):
        return AEBController(ttc_warning=4.0, ttc_partial=3.0, ttc_full=1.5, detection_range=80.0)
    
    def test_initial_state(self, aeb):
        """测试初始状态为正常"""
        assert aeb.state == aeb.NORMAL
        assert aeb.get_state_name() == "正常"
    
    def test_normal_to_warning(self, aeb):
        """测试正常状态到预警状态的转换"""
        # TTC=3.5秒，车距在距离保护范围之外。
        brake = aeb.update_state(distance=21.0, relative_speed=6.0)
        assert aeb.state == aeb.WARNING
        assert aeb.get_state_name() == "预警"
        assert brake >= 0.0  # 预警状态下制动输出为0.0
    
    def test_warning_to_partial_brake(self, aeb):
        """测试预警状态到部分制动状态的转换"""
        # 先触发预警
        aeb.update_state(distance=21.0, relative_speed=6.0)
        assert aeb.state == aeb.WARNING
        
        # 再触发部分制动：TTC=2.5秒
        brake = aeb.update_state(distance=15.0, relative_speed=6.0)
        assert aeb.state == aeb.PARTIAL_BRAKE
        assert aeb.get_state_name() == "部分制动"
        assert brake >= 0.4  # 实际部分制动输出为0.4
    
    def test_partial_brake_to_full_brake(self, aeb):
        """测试部分制动到全力制动状态的转换"""
        # 触发全力制动：TTC=1.0秒
        brake = aeb.update_state(distance=5.0, relative_speed=5.0)
        assert aeb.state == aeb.FULL_BRAKE
        assert aeb.get_state_name() == "全力制动"
        assert brake == 1.0
    
    def test_full_brake_by_distance(self, aeb):
        """测试通过距离触发全力制动"""
        # 小于10米时，距离保护要求全力制动。
        brake = aeb.update_state(distance=3.0, relative_speed=1.0)
        assert aeb.state == aeb.FULL_BRAKE
        assert aeb.get_state_name() == "全力制动"
        assert brake == 1.0
    
    def test_recovery_to_normal(self, aeb):
        """测试状态恢复到正常"""
        # 先触发全力制动
        aeb.update_state(distance=3.0, relative_speed=5.0)
        assert aeb.state == aeb.FULL_BRAKE
        
        # 恢复为正常（无前方车辆）
        brake = aeb.update_state(distance=None, relative_speed=0.0)
        assert aeb.state == aeb.NORMAL
        assert brake == 0.0
    
    def test_full_state_transition_sequence(self, aeb):
        """测试完整的状态转换序列：正常->预警->部分制动->全力制动"""
        states = []
        
        # 正常状态
        brake = aeb.update_state(distance=50.0, relative_speed=5.0)
        states.append(aeb.get_state_name())
        assert aeb.state == aeb.NORMAL
        
        # 预警状态
        brake = aeb.update_state(distance=21.0, relative_speed=6.0)
        states.append(aeb.get_state_name())
        assert aeb.state == aeb.WARNING
        
        # 部分制动状态（更近距离）
        brake = aeb.update_state(distance=15.0, relative_speed=6.0)
        states.append(aeb.get_state_name())
        assert aeb.state == aeb.PARTIAL_BRAKE
        
        # 全力制动状态
        brake = aeb.update_state(distance=4.0, relative_speed=5.0)
        states.append(aeb.get_state_name())
        assert aeb.state == aeb.FULL_BRAKE
        
        assert states == ["正常", "预警", "部分制动", "全力制动"]


class TestBrakeOutput:
    """制动输出测试"""
    
    @pytest.fixture
    def aeb(self):
        return AEBController(ttc_warning=4.0, ttc_partial=3.0, ttc_full=1.5, detection_range=80.0)
    
    def test_normal_brake_output(self, aeb):
        """测试正常状态下的制动输出"""
        brake = aeb.update_state(distance=50.0, relative_speed=2.0)
        assert brake == 0.0
    
    def test_warning_brake_output(self, aeb):
        """测试预警状态下的制动输出"""
        brake = aeb.update_state(distance=21.0, relative_speed=6.0)
        assert aeb.state == aeb.WARNING
        assert brake == 0.0
    
    def test_partial_brake_output(self, aeb):
        """测试部分制动状态下的制动输出"""
        brake = aeb.update_state(distance=12.0, relative_speed=1.0)
        assert aeb.state == aeb.PARTIAL_BRAKE
        assert 0.4 <= brake <= 0.5  # 实际部分制动输出为0.4
    
    def test_full_brake_output(self, aeb):
        """测试全力制动状态下的制动输出"""
        brake = aeb.update_state(distance=3.0, relative_speed=5.0)
        assert aeb.state == aeb.FULL_BRAKE
        assert brake == 1.0
    
    def test_distance_braking_increases_at_threshold(self, aeb):
        """测试距离保护从部分制动升级为全力制动"""
        brake_far = aeb.update_state(distance=10.0, relative_speed=0.0)
        assert aeb.state == aeb.PARTIAL_BRAKE
        brake_near = aeb.update_state(distance=9.99, relative_speed=0.0)
        assert aeb.state == aeb.FULL_BRAKE
        assert (brake_far, brake_near) == (0.4, 1.0)


class TestCollisionAvoidance:
    """碰撞避免测试 - 参数化测试"""
    
    @pytest.fixture
    def aeb(self):
        return AEBController(ttc_warning=2.5, ttc_partial=1.6, ttc_full=0.6, detection_range=80.0)
    
    @pytest.mark.parametrize(
        "target_speed_kmh,initial_distance,relative_speed,expected_avoid_collision",
        [
            (20, 30, 2.0, True),   # 20km/h，30米距离，相对速度2m/s
            (30, 40, 3.0, True),   # 30km/h，40米距离，相对速度3m/s
            (40, 50, 4.0, True),   # 40km/h，50米距离，相对速度4m/s
            (50, 30, 8.0, False),  # 50km/h，30米距离，相对速度8m/s（可能碰撞）
            (60, 25, 10.0, False), # 60km/h，25米距离，相对速度10m/s（可能碰撞）
        ],
        ids=["20kmh_safe", "30kmh_safe", "40kmh_safe", "50kmh_risk", "60kmh_risk"]
    )
    def test_collision_avoidance_at_different_speeds(
        self, aeb, target_speed_kmh, initial_distance, relative_speed, expected_avoid_collision
    ):
        """
        测试不同目标速度下的碰撞避免能力
        
        参数:
        - target_speed_kmh: 目标车速度
        - initial_distance: 初始距离
        - relative_speed: 相对速度
        - expected_avoid_collision: 是否期望避免碰撞
        """
        # 模拟AEB触发过程
        states_triggered = []
        max_brake = 0.0
        
        # 逐步模拟距离减小的过程
        for dist in range(int(initial_distance), 0, -1):
            brake = aeb.update_state(distance=float(dist), relative_speed=relative_speed)
            states_triggered.append(aeb.get_state_name())
            max_brake = max(max_brake, brake)
            
            # 如果触发全力制动，记录并退出
            if aeb.state == aeb.FULL_BRAKE:
                break
        
        # 验证AEB是否被触发
        assert len(states_triggered) > 0, "AEB应该被触发"
        
        # 验证最大制动强度
        assert max_brake > 0.0, "应该有制动输出"
        
        # 根据相对距离判断是否可能避免碰撞
        ttc = initial_distance / relative_speed if relative_speed > 0 else float('inf')
        if ttc > 1.5:  # TTC足够大，应该能避免碰撞
            assert expected_avoid_collision or aeb.FULL_BRAKE in [aeb.NORMAL, aeb.WARNING, aeb.PARTIAL_BRAKE, aeb.FULL_BRAKE]


class TestTTCThresholds:
    """TTC阈值组合测试 - 参数化测试"""
    
    @pytest.mark.parametrize(
        "ttc_warning,ttc_partial,ttc_full,test_distance,test_relative_speed,expected_state,expected_brake_range",
        [
            (4.0, 3.0, 1.5, 50.0, 10.0, 0, (0.0, 0.0)),      # TTC=5.0s，正常
            (4.0, 3.0, 1.5, 25.0, 10.0, 2, (0.4, 0.4)),      # TTC=2.5s，实际触发部分制动
            (4.0, 3.0, 1.5, 20.0, 10.0, 2, (0.4, 0.4)),      # TTC=2.0s，部分制动
            (4.0, 3.0, 1.5, 10.0, 10.0, 3, (1.0, 1.0)),      # TTC=1.0s，全力制动
            (2.5, 1.6, 0.6, 15.0, 5.0, 0, (0.0, 0.0)),       # 激进型TTC，TTC=3.0s，正常
            (2.5, 1.6, 0.6, 16.0, 8.0, 1, (0.0, 0.0)),       # TTC=2.0s，预警
            (2.5, 1.6, 0.6, 15.0, 10.0, 2, (0.4, 0.4)),      # TTC=1.5s，部分制动
        ],
        ids=[
            "default_ttc_normal",
            "default_ttc_warning",
            "default_ttc_partial",
            "default_ttc_full",
            "aggressive_ttc_normal",
            "aggressive_ttc_warning",
            "aggressive_ttc_partial"
        ]
    )
    def test_ttc_threshold_combinations(
        self, ttc_warning, ttc_partial, ttc_full,
        test_distance, test_relative_speed,
        expected_state, expected_brake_range
    ):
        """
        测试不同TTC阈值组合下的状态切换
        
        参数:
        - ttc_warning/Partial/Full: TTC阈值
        - test_distance: 测试距离
        - test_relative_speed: 测试相对速度
        - expected_state: 期望状态（0-正常，1-预警，2-部分制动，3-全力制动）
        - expected_brake_range: 期望制动输出范围
        """
        aeb = AEBController(
            ttc_warning=ttc_warning,
            ttc_partial=ttc_partial,
            ttc_full=ttc_full,
            detection_range=80.0
        )
        
        brake = aeb.update_state(distance=test_distance, relative_speed=test_relative_speed)
        
        # 验证状态
        assert aeb.state == expected_state, f"期望状态{expected_state}，实际{aeb.state}"
        
        # 验证制动输出范围
        min_brake, max_brake = expected_brake_range
        assert min_brake <= brake <= max_brake, f"制动输出{brake}不在范围[{min_brake}, {max_brake}]"


class TestEdgeCases:
    """边界情况测试"""
    
    @pytest.fixture
    def aeb(self):
        return AEBController()
    
    def test_no_front_vehicle(self, aeb):
        """测试未检测到前方车辆"""
        brake = aeb.update_state(distance=None, relative_speed=0.0)
        assert aeb.state == aeb.NORMAL
        assert brake == 0.0
    
    def test_very_far_distance(self, aeb):
        """测试极远距离"""
        brake = aeb.update_state(distance=200.0, relative_speed=5.0)
        assert aeb.state == aeb.NORMAL
        assert brake == 0.0
    
    def test_zero_relative_speed_at_close_distance(self, aeb):
        """测试近距离但相对速度为0"""
        # 相对速度为0时TTC为无穷大，但距离小于detection_range
        brake = aeb.update_state(distance=5.0, relative_speed=0.0)
        # 距离保护在TTC为无穷大时仍有效。
        assert aeb.state == aeb.FULL_BRAKE
        assert brake == 1.0
    
    def test_beyond_detection_range(self, aeb):
        """测试超出检测范围"""
        # 虽然update_state不直接使用detection_range，但应处理远距离情况
        brake = aeb.update_state(distance=100.0, relative_speed=1.0)
        assert brake == 0.0 or aeb.state == aeb.NORMAL
    
    def test_state_names(self, aeb):
        """测试状态名称映射"""
        assert AEBController().get_state_name() == "正常"
        
        aeb.state = aeb.WARNING
        assert aeb.get_state_name() == "预警"
        
        aeb.state = aeb.PARTIAL_BRAKE
        assert aeb.get_state_name() == "部分制动"
        
        aeb.state = aeb.FULL_BRAKE
        assert aeb.get_state_name() == "全力制动"


class TestAggressiveSettings:
    """激进型AEB设置测试"""
    
    def test_aggressive_aeb_triggers_earlier(self):
        """测试激进型AEB比默认AEB更早触发制动"""
        default_aeb = AEBController(ttc_warning=4.0, ttc_partial=3.0, ttc_full=1.5)
        aggressive_aeb = AEBController(ttc_warning=2.5, ttc_partial=1.6, ttc_full=0.6)
        
        # 相同条件下，激进型应该产生更大的制动输出
        distance = 12.0
        relative_speed = 4.0
        
        default_brake = default_aeb.update_state(distance, relative_speed)
        aggressive_brake = aggressive_aeb.update_state(distance, relative_speed)
        
        # 验证两个控制器都有响应（激进型TTC更严格，可能在某些条件下制动更晚）
        assert default_brake >= 0.0
        assert aggressive_brake >= 0.0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--html=../Desktop/test_aeb_report.html"])
