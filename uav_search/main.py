"""
无人机协同搜索系统主程序入口

该模块是整个系统的启动入口，负责：
1. 解析命令行参数
2. 加载配置和场景
3. 初始化系统各模块
4. 运行仿真主循环
5. 输出结果和可视化

使用方式：
    python -m uav_search.main --config config/default.yaml --scenario config/scenarios/area_search_1uav.yaml --output runs/area_search_1uav_snapshots.json
"""
from __future__ import annotations

import argparse
from pathlib import Path

from uav_search.core.config import load_config, validate_config
from uav_search.core.data_types import DecisionOutput
from uav_search.core.scheduler import Scheduler
from uav_search.evaluation.metrics import compute_metrics, save_metrics
from uav_search.maps.map_loader import build_grid_map
from uav_search.simulation.scenario_events import ScenarioEventInjector
from uav_search.simulation.simulator import Simulator
from uav_search.uav.fleet_manager import FleetManager
from uav_search.visualization.report_generator import generate_report_charts
from uav_search.visualization.static_viewer import render_static_map


def run(
    default_config: Path,
    scenario_path: Path,
    output_path: Path,
    image_path: Path | None = None,
    metrics_path: Path | None = None,
    report_dir: Path | None = None,
    play: bool = False,
    play_interval_ms: int = 160,
    play_repeat: bool = False,
) -> DecisionOutput:
    """运行完整的仿真流程

    这是系统的核心运行函数，执行完整的仿真流程：
    初始化 → 首次决策 → 仿真循环 → 结果输出

    参数：
        default_config: 默认配置文件路径
        scenario_path: 场景配置文件路径
        output_path: 快照输出文件路径
        image_path: 静态可视化图片路径（可选）
        metrics_path: 评估指标输出路径（可选）
        report_dir: 报告图表输出目录（可选）
        play: 是否播放实时动画
        play_interval_ms: 播放帧间隔（毫秒）
        play_repeat: 是否循环播放

    返回：
        DecisionOutput: 最终决策输出结果

    流程概述：
        1. 加载并验证配置
        2. 构建栅格地图
        3. 创建无人机编队
        4. 初始化调度器
        5. 执行首次决策（生成初始任务）
        6. 运行仿真主循环
        7. 输出结果和可视化
    """
    # 步骤1: 加载并验证配置
    config = load_config(default_config, scenario_path)
    validate_config(config)
    scenario = config.get("scenario", {})

    # 步骤2: 构建栅格地图
    # 根据配置创建栅格地图，加载障碍物、禁飞区、重点区域等
    grid_map = build_grid_map(config)

    # 步骤3: 创建无人机编队
    # 根据配置和场景创建无人机编队
    fleet = FleetManager.from_config(config, scenario)

    # 步骤4: 初始化调度器
    # 调度器是决策核心，协调任务分配、路径规划等
    scheduler = Scheduler(grid_map, fleet, config)

    # 步骤5: 执行首次决策
    # 生成初始搜索任务，分配给无人机，规划路径
    decision_output = scheduler.regular_cycle(now=0.0)

    # 步骤6: 运行仿真主循环
    # 创建仿真器，执行时间步进仿真
    simulator = Simulator(grid_map, fleet, config)
    simulator.record_snapshot(scheduler=scheduler)  # 记录初始状态快照

    # 创建场景事件注入器（用于注入预设事件，如目标发现、地图更新等）
    event_injector = ScenarioEventInjector(scenario.get("events", []))

    # 运行仿真主循环
    simulator.run(scheduler=scheduler, event_injector=event_injector)

    # 步骤7: 输出结果和可视化
    # 保存仿真快照
    simulator.save_snapshots(output_path, run_id=scenario.get("name", "manual_run"))

    # 计算并保存评估指标
    if metrics_path is not None:
        metrics = compute_metrics(
            scenario.get("name", "manual_run"),
            grid_map,
            fleet,
            simulator.snapshots,
            mission_complete_coverage_threshold=float(config["search"].get("mission_complete_coverage_threshold", 0.95)),
        )
        save_metrics(metrics, metrics_path)

    # 生成静态可视化图片
    if image_path is not None:
        render_static_map(
            grid_map,
            fleet.get_all_states(),
            image_path,
            title=f"{scenario.get('name', 'UAV Search')} final state",
            snapshots=simulator.snapshots,
        )

    # 生成报告图表（覆盖率曲线、轨迹图等）
    if report_dir is not None:
        generate_report_charts(simulator.snapshots, report_dir)

    # 播放实时动画
    if play:
        from uav_search.visualization.realtime_viewer import play_snapshots

        play_snapshots(
            grid_map,
            simulator.snapshots,
            sensor_radius_cells=int(config["uav"]["sensor_radius_cells"]),
            interval_ms=play_interval_ms,
            repeat=play_repeat,
        )

    # 返回最终决策输出
    return DecisionOutput(
        timestamp=simulator.time_s,
        commands=decision_output.commands,
        assignments=decision_output.assignments,
        events_handled=decision_output.events_handled,
        global_coverage=grid_map.coverage_rate(),
        priority_coverage=grid_map.coverage_rate(priority_only=True),
        decision_latency_ms=decision_output.decision_latency_ms,
    )


def main() -> None:
    """主函数入口

    解析命令行参数并运行仿真。

    命令行参数：
        --config: 配置文件路径，默认 config/default.yaml
        --scenario: 场景文件路径，默认 config/scenarios/area_search_1uav.yaml
        --output: 快照输出路径，默认 runs/area_search_1uav_snapshots.json
        --image: 静态可视化图片路径（可选）
        --metrics: 评估指标输出路径（可选）
        --report-dir: 报告图表输出目录（可选）
        --play: 是否播放实时动画
        --play-interval-ms: 播放帧间隔（毫秒）
        --play-repeat: 是否循环播放
    """
    parser = argparse.ArgumentParser(description="Run the first-loop UAV search simulation.")
    parser.add_argument("--config", type=Path, default=Path("config/default.yaml"))
    parser.add_argument("--scenario", type=Path, default=Path("config/scenarios/area_search_1uav.yaml"))
    parser.add_argument("--output", type=Path, default=Path("runs/area_search_1uav_snapshots.json"))
    parser.add_argument("--image", type=Path, default=None, help="Optional PNG path for a static visualization.")
    parser.add_argument("--metrics", type=Path, default=None, help="Optional JSON path for evaluation metrics.")
    parser.add_argument("--report-dir", type=Path, default=None, help="Optional directory for report charts.")
    parser.add_argument("--play", action="store_true", help="Open a realtime matplotlib playback window after running.")
    parser.add_argument("--play-interval-ms", type=int, default=160, help="Playback frame interval in milliseconds.")
    parser.add_argument("--play-repeat", action="store_true", help="Loop playback until the window is closed.")
    args = parser.parse_args()

    # 运行仿真
    output = run(
        args.config,
        args.scenario,
        args.output,
        args.image,
        args.metrics,
        args.report_dir,
        args.play,
        args.play_interval_ms,
        args.play_repeat,
    )

    # 输出最终结果摘要
    print(
        f"finished timestamp={output.timestamp:.1f}s "
        f"coverage={output.global_coverage:.3f} "
        f"priority_coverage={output.priority_coverage:.3f} "
        f"decision_latency_ms={output.decision_latency_ms:.2f}"
    )


if __name__ == "__main__":
    main()
