# 多无人机区域协同搜索决策模型

当前仓库已完成首轮可运行闭环：

- YAML 配置和基础场景加载
- 核心枚举与 dataclass 数据结构
- 栅格地图、障碍物、禁飞区、重点区域建模
- UAV 状态、电量消耗、路径跟随
- A* 路径规划
- 搜索任务生成、均衡连通区域划分、Boustrophedon 覆盖航路点
- 顺序单物品拍卖任务分配
- 固定周期调度器
- 基础冲突检测与等待消解
- 事件管理器与低电量/离线高优先级事件处理
- 基础仿真步进和覆盖率更新
- `snapshots.json` 仿真快照输出
- 地图、UAV、A* 基础单元测试

## 环境

建议使用 Python 3.10+。

```powershell
pip install -r requirements.txt
```

本机已验证可使用：

```powershell
E:\anaconda\python.exe -m pytest -q
E:\anaconda\python.exe -m uav_search.main --config config/default.yaml --scenario config/scenarios/basic.yaml --output runs/basic_snapshots.json
```

## 运行测试

```powershell
pytest -q
```

## 运行基础仿真

```powershell
python -m uav_search.main --config config/default.yaml --scenario config/scenarios/basic.yaml --output runs/basic_snapshots.json --image runs/basic_view.png --metrics runs/basic_metrics.json --report-dir runs/basic_report
```

运行完成后会输出仿真时间、覆盖率、重点区域覆盖率和首条路径规划耗时，并在 `runs/basic_snapshots.json` 写入快照，在 `runs/basic_view.png` 输出静态效果图，在 `runs/basic_metrics.json` 输出基础评估指标。

## 运行多 UAV 基础场景

```powershell
python -m uav_search.main --config config/default.yaml --scenario config/scenarios/multi_basic.yaml --output runs/multi_basic_snapshots.json --image runs/multi_basic_view.png --metrics runs/multi_basic_metrics.json --report-dir runs/multi_basic_report
```

该场景会生成多个搜索任务，通过顺序拍卖分配给 3 架 UAV，并输出覆盖率快照和静态效果图。

`--report-dir` 会额外输出：

- `coverage_curve.png`
- `uav_trajectories.png`
- `event_timeline.png`

## 批量运行多个场景

```powershell
python -m uav_search.experiments.run_batch --scenarios basic multi_basic dynamic_basic --output-dir runs/batch_001
```

每个场景会输出独立的 `snapshots.json`、`metrics.json`、`final_view.png` 和报告图表，批量目录下还会生成 `summary.json` 与 `summary.csv`。

## 当前动态响应能力

调度器当前支持以下高优先级事件：

- `LOW_BATTERY`：无人机切换为返航，并规划返回 home 的路径
- `UAV_OFFLINE`：无人机标记为离线并停止执行路径
- `MAP_UPDATE`：运行时更新地图，并对失效路径触发局部重规划
- `TARGET_FOUND`：发现者切换为确认状态，并重规划至目标位置

场景文件中的 `events` 会按 `time_s` 在仿真过程中自动注入。例如 `config/scenarios/dynamic_basic.yaml` 会在运行中注入 `TARGET_FOUND` 和 `MAP_UPDATE`。

## 文档

- `docs/superpowers/specs/2026-06-11-multi-uav-search-design.md`：总体设计方案
- `docs/superpowers/specs/2026-06-11-implementation-plan.md`：详细实现计划
- `docs/superpowers/specs/2026-06-11-development-readiness-spec.md`：开发前准备与接口规格
