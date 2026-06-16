"""
无人机编队管理模块

管理多无人机编队，提供编队级别的操作接口。
负责无人机的创建、状态查询、路径分配和运动控制。

主要功能：
- 从配置创建无人机编队
- 管理编队中所有无人机的状态
- 提供编队级别的查询和操作接口
- 协调编队的运动控制
"""
from __future__ import annotations

from collections.abc import Iterable

from uav_search.core.data_types import Position, UAVState, UAVStatus
from uav_search.uav.uav_model import UAV


class FleetManager:
    """无人机编队管理器

    管理多架无人机的集合，提供统一的操作接口。

    属性：
        _uavs: 无人机字典，键为无人机ID

    主要职责：
        - 维护编队中所有无人机的状态
        - 提供无人机查询和访问接口
        - 协调路径分配和状态切换
        - 推进编队的运动仿真
    """

    def __init__(self, uavs: Iterable[UAV]) -> None:
        """初始化编队管理器

        参数：
            uavs: 无人机对象的可迭代集合
        """
        self._uavs = {uav.state.id: uav for uav in uavs}

    @classmethod
    def from_config(cls, config: dict, scenario: dict | None = None) -> "FleetManager":
        """从配置创建编队管理器

        根据配置和场景参数创建无人机编队。
        支持场景自定义无人机初始位置和电量。

        参数：
            config: 系统配置字典
            scenario: 场景配置字典，可包含自定义无人机参数

        返回：
            FleetManager: 创建的编队管理器对象

        创建逻辑：
            - 如果场景指定了无人机列表，按场景参数创建
            - 否则按系统配置创建指定数量的无人机
            - 所有无人机初始状态为IDLE
        """
        uav_config = config["uav"]
        scenario_uavs = (scenario or {}).get("uavs", [])
        uavs: list[UAV] = []

        if scenario_uavs:
            # 场景自定义无人机配置
            for item in scenario_uavs:
                home = Position(int(item["home_position"][0]), int(item["home_position"][1]))
                initial = Position(int(item["initial_position"][0]), int(item["initial_position"][1]))
                state = UAVState(
                    id=str(item["id"]),
                    position=initial,
                    velocity_mps=float(uav_config["max_speed_mps"]),
                    heading_deg=0.0,
                    battery=float(item.get("battery", 1.0)),
                    sensor_radius_cells=int(uav_config["sensor_radius_cells"]),
                    status=UAVStatus.IDLE,
                    home_position=home,
                )
                uavs.append(UAV(state, endurance_s=float(uav_config["endurance_s"])))
        else:
            # 默认配置：所有无人机从同一位置起飞
            home = Position(int(uav_config["home_position"][0]), int(uav_config["home_position"][1]))
            for idx in range(int(uav_config["count"])):
                state = UAVState(
                    id=f"uav_{idx + 1:02d}",
                    position=home,
                    velocity_mps=float(uav_config["max_speed_mps"]),
                    heading_deg=0.0,
                    battery=1.0,
                    sensor_radius_cells=int(uav_config["sensor_radius_cells"]),
                    status=UAVStatus.IDLE,
                    home_position=home,
                )
                uavs.append(UAV(state, endurance_s=float(uav_config["endurance_s"])))

        return cls(uavs)

    def get_uav(self, uav_id: str) -> UAV:
        """获取指定无人机对象

        参数：
            uav_id: 无人机ID

        返回：
            UAV: 无人机对象

        异常：
            KeyError: 如果无人机ID不存在
        """
        return self._uavs[uav_id]

    def get_all_uavs(self) -> list[UAV]:
        """获取所有无人机对象

        返回：
            list[UAV]: 所有无人机对象列表
        """
        return list(self._uavs.values())

    def get_all_states(self) -> list[UAVState]:
        """获取所有无人机状态

        返回：
            list[UAVState]: 所有无人机状态列表
        """
        return [uav.state for uav in self._uavs.values()]

    def get_available_uavs(self) -> list[UAVState]:
        """获取所有可用无人机状态

        可用条件：available=True 且 status=IDLE

        返回：
            list[UAVState]: 可用无人机状态列表

        用途：
            用于任务分配时选择候选无人机
        """
        return [uav.state for uav in self._uavs.values() if uav.state.available and uav.state.status == UAVStatus.IDLE]

    def assign_path(self, uav_id: str, path: list[Position], status: UAVStatus = UAVStatus.SEARCHING) -> None:
        """为指定无人机分配路径

        参数：
            uav_id: 无人机ID
            path: 路径点列表
            status: 分配后的状态，默认为SEARCHING

        用途：
            任务分配后为无人机设置执行路径
        """
        self.get_uav(uav_id).assign_path(path, status=status)

    def set_status(self, uav_id: str, status: UAVStatus) -> None:
        """设置无人机状态

        参数：
            uav_id: 无人机ID
            status: 新状态

        注意：
            - 设置为OFFLINE时，available自动设为False
            - 其他状态时，available设为True
        """
        state = self.get_uav(uav_id).state
        state.status = status
        state.available = status != UAVStatus.OFFLINE

    def step(self, time_step_s: float, resolution_m: float) -> None:
        """推进编队运动仿真一个时间步

        对编队中所有无人机执行运动更新。

        参数：
            time_step_s: 时间步长（秒）
            resolution_m: 栅格分辨率（米）

        用途：
            在仿真主循环中调用，推进所有无人机的位置更新
        """
        for uav in self._uavs.values():
            uav.move_along_path(time_step_s, resolution_m)
