# 多无人机区域协同搜索决策模型

当前仓库已完成首轮可运行闭环：

- YAML 配置和基础场景加载
- 核心枚举与 dataclass 数据结构
- 栅格地图、障碍物、禁飞区、重点区域建模
- UAV 状态、电量消耗、路径跟随
- A* 路径规划
- 搜索任务生成、简单区域划分、Boustrophedon 覆盖航路点
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
python -m uav_search.main --config config/default.yaml --scenario config/scenarios/basic.yaml --output runs/basic_snapshots.json
```

运行完成后会输出仿真时间、覆盖率、重点区域覆盖率和首条路径规划耗时，并在 `runs/basic_snapshots.json` 写入快照。

## 运行多 UAV 基础场景

```powershell
python -m uav_search.main --config config/default.yaml --scenario config/scenarios/multi_basic.yaml --output runs/multi_basic_snapshots.json
```

该场景会生成多个搜索任务，通过顺序拍卖分配给 3 架 UAV，并输出覆盖率快照。

## 文档

- `docs/superpowers/specs/2026-06-11-multi-uav-search-design.md`：总体设计方案
- `docs/superpowers/specs/2026-06-11-implementation-plan.md`：详细实现计划
- `docs/superpowers/specs/2026-06-11-development-readiness-spec.md`：开发前准备与接口规格
