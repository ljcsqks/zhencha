# 多无人机区域协同搜索决策模型 - 开发前准备与接口规格

本文档用于补充 `2026-06-11-multi-uav-search-design.md` 和 `2026-06-11-implementation-plan.md`。前两份文档定义系统目标、模块划分和阶段计划；本文档进一步明确正式开发前必须对齐的工程约定，包括坐标单位、核心数据结构、输入输出协议、模块接口、状态机、配置 schema、错误处理、日志格式和测试矩阵。

## 1. 开发前准入清单

正式进入代码开发前，应完成以下准备工作：

| 类别 | 准入项 | 目标 |
|------|--------|------|
| 工程环境 | 确认 Python 版本为 3.10+ | 支持现代类型标注和 dataclass |
| 依赖策略 | 确认基础依赖和可选依赖 | 避免开发中途反复改算法实现边界 |
| 包结构 | 确认顶层包名和模块目录 | 保证导入路径稳定 |
| 数据契约 | 确认核心 dataclass 和枚举 | 避免模块间使用不同数据形态 |
| 协议格式 | 确认输入、输出、事件、日志 JSON 格式 | 为后续接入外部系统预留边界 |
| 配置格式 | 确认默认配置和场景配置 schema | 保证仿真可复现、参数可调 |
| 状态机 | 确认 UAV、Task、Event 状态转移 | 避免调度逻辑出现隐式分支 |
| 错误处理 | 确认路径失败、任务不可达、冲突无法消解等策略 | 避免算法失败时系统停滞 |
| 测试矩阵 | 确认阶段性单元测试和集成测试 | 每阶段均有可验收标准 |

## 2. 技术栈与依赖约定

### 2.1 基础技术栈

- Python: `3.10+`
- 包管理：初期可使用 `requirements.txt`，后续如项目复杂度上升可迁移到 `pyproject.toml`
- 测试框架：`pytest`
- 配置文件：YAML
- 日志与实验结果：JSON 或 CSV，优先 JSON
- 可视化：`matplotlib`

### 2.2 必需依赖

```text
numpy
matplotlib
pyyaml
pytest
```

### 2.3 可选依赖

以下依赖不作为第一阶段强制项，只有在明确需要时再引入：

| 依赖 | 用途 | 是否建议初期引入 |
|------|------|------------------|
| scikit-learn | K-Means 区域划分 | 可选。若希望减少自实现成本，可引入 |
| shapely | 多边形禁飞区、重点区栅格化 | 可选。若场景多边形复杂，建议引入 |
| pandas | 实验结果表格化分析 | 后期评估阶段可引入 |

若不引入 `scikit-learn`，K-Means 使用 numpy 自实现；若不引入 `shapely`，多边形点内判断使用射线法自实现。

## 3. 坐标、单位与命名约定

### 3.1 坐标体系

系统同时支持世界坐标和栅格坐标，但算法核心统一使用栅格坐标。

| 坐标类型 | 类型 | 单位 | 使用场景 |
|----------|------|------|----------|
| 世界坐标 | float | meter | 场景配置、外部输入、可视化标尺 |
| 栅格坐标 | int | grid cell | 地图、路径规划、任务分配、覆盖率计算 |

转换规则：

```python
grid_x = floor(world_x / resolution)
grid_y = floor(world_y / resolution)
world_x = (grid_x + 0.5) * resolution
world_y = (grid_y + 0.5) * resolution
```

约定：

- 地图原点为左下角或数组左上角必须在可视化中统一。建议内部数组使用 `(x, y)` 逻辑坐标，对 matplotlib 绘制时做必要转换。
- 所有核心 dataclass 中的 `position`、`path`、`target` 默认表示栅格坐标。
- 场景 YAML 中的几何区域默认使用世界坐标，加载时转换为栅格。

### 3.2 单位约定

| 字段 | 单位 | 范围 |
|------|------|------|
| `time` / `timestamp` | second | `>= 0` |
| `time_step` | second | `> 0` |
| `distance` | meter 或 grid cells，字段名需明确 | 不混用 |
| `velocity` / `max_speed` | m/s | `> 0` |
| `battery` | ratio | `[0.0, 1.0]` |
| `heading` | degree | `[0, 360)` |
| `search_confidence` | ratio | `[0.0, 1.0]` |
| `priority` | float | 值越大优先级越高 |

命名要求：

- 以米为单位的字段添加 `_m` 或在字段说明中明确，例如 `width_m`。
- 以栅格为单位的字段添加 `_cells` 或使用 `Position`。
- 时间延迟指标使用 `_ms` 或 `_s` 后缀。

## 4. 核心枚举

```python
class CellType(Enum):
    FREE = "FREE"
    OBSTACLE = "OBSTACLE"
    NO_FLY = "NO_FLY"
    PRIORITY = "PRIORITY"

class UAVStatus(Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    CONFIRMING = "CONFIRMING"
    RETURNING = "RETURNING"
    AVOIDING = "AVOIDING"
    OFFLINE = "OFFLINE"

class TaskType(Enum):
    SEARCH = "SEARCH"
    CONFIRM = "CONFIRM"
    RETURN = "RETURN"

class TaskStatus(Enum):
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    BLOCKED = "BLOCKED"
    CANCELLED = "CANCELLED"

class EventType(Enum):
    TARGET_FOUND = "TARGET_FOUND"
    TARGET_APPEAR = "TARGET_APPEAR"
    CONFIRM_DONE = "CONFIRM_DONE"
    UAV_OFFLINE = "UAV_OFFLINE"
    UAV_RECOVERED = "UAV_RECOVERED"
    MAP_UPDATE = "MAP_UPDATE"
    LOW_BATTERY = "LOW_BATTERY"
    TASK_BLOCKED = "TASK_BLOCKED"
    CONFLICT_DETECTED = "CONFLICT_DETECTED"

class EventPriority(Enum):
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4

class CommandType(Enum):
    HOLD = "HOLD"
    FOLLOW_PATH = "FOLLOW_PATH"
    CONFIRM_TARGET = "CONFIRM_TARGET"
    RETURN_HOME = "RETURN_HOME"
    REPLAN = "REPLAN"
```

## 5. 核心数据结构

所有跨模块传递的数据优先使用 `dataclass`，避免直接传递裸 `dict`。外部输入输出可以使用 JSON，在系统边界转换为 dataclass。

### 5.1 Position

```python
@dataclass(frozen=True)
class Position:
    x: int
    y: int
```

约定：

- `Position` 表示栅格坐标。
- `Position` 应可 hash，用于 set、dict key、路径去重和区域集合。

### 5.2 GridCell

```python
@dataclass
class GridCell:
    position: Position
    cell_type: CellType
    passable: bool
    search_confidence: float = 0.0
    search_priority: float = 1.0
    last_search_time: float | None = None
    target_ids: list[str] = field(default_factory=list)
```

约束：

- `OBSTACLE` 和 `NO_FLY` 默认 `passable=False`。
- `PRIORITY` 默认 `passable=True`，除非同时被障碍物或禁飞区覆盖。
- 若多种区域重叠，优先级为：`NO_FLY > OBSTACLE > PRIORITY > FREE`。

### 5.3 GridMap

`GridMap` 内部可使用 numpy 多层数组存储，不要求每次查询都创建 `GridCell` 对象。

关键属性：

```python
width_m: float
height_m: float
resolution_m: float
width_cells: int
height_cells: int
terrain: np.ndarray
passable: np.ndarray
search_confidence: np.ndarray
search_priority: np.ndarray
last_search_time: np.ndarray
coverage_count: np.ndarray
```

核心方法：

```python
in_bounds(pos: Position) -> bool
is_passable(pos: Position) -> bool
get_cell(pos: Position) -> GridCell
set_cell(pos: Position, attrs: dict[str, Any]) -> None
get_neighbors(pos: Position, mode: int = 8) -> list[Position]
get_searchable_cells() -> list[Position]
get_unsearched_cells(threshold: float) -> list[Position]
mark_covered(center: Position, radius_cells: int, timestamp: float) -> list[Position]
decay_search_confidence(current_time: float, lambda_decay: float) -> None
world_to_grid(x_m: float, y_m: float) -> Position
grid_to_world(pos: Position) -> tuple[float, float]
```

### 5.4 UAVState

```python
@dataclass
class UAVState:
    id: str
    position: Position
    velocity_mps: float
    heading_deg: float
    battery: float
    sensor_radius_cells: int
    status: UAVStatus
    home_position: Position
    current_task_id: str | None = None
    path: list[Position] = field(default_factory=list)
    path_index: int = 0
    available: bool = True
    assigned_task_count: int = 0
    total_distance_m: float = 0.0
    effective_search_distance_m: float = 0.0
```

约束：

- `battery` 使用 `[0.0, 1.0]`。
- `available=False` 时不参与任务拍卖。
- `OFFLINE` 状态必须同时设置 `available=False`。
- `path_index` 指向当前正在执行或下一步要执行的路径点。

### 5.5 Task

```python
@dataclass
class Task:
    id: str
    type: TaskType
    priority: float
    target_cells: set[Position]
    entry_point: Position
    status: TaskStatus = TaskStatus.PENDING
    assigned_uav_id: str | None = None
    waypoints: list[Position] = field(default_factory=list)
    estimated_cost_m: float = 0.0
    created_at: float = 0.0
    updated_at: float = 0.0
    progress: float = 0.0
    source_event_id: str | None = None
```

约束：

- `SEARCH` 任务必须有 `target_cells` 和 `waypoints`。
- `CONFIRM` 任务通常只有一个目标点，优先分配给发现目标的 UAV。
- `RETURN` 任务由低电量或任务结束触发，目标为 `home_position`。

### 5.6 Target

```python
@dataclass
class Target:
    id: str
    position: Position
    target_type: str
    confidence: float
    first_seen_time: float
    discovered_by: str | None = None
    confirmed: bool = False
    confirmed_time: float | None = None
```

### 5.7 Event

```python
@dataclass(order=True)
class Event:
    sort_key: tuple[int, float] = field(init=False, repr=False)
    id: str
    type: EventType
    timestamp: float
    priority: EventPriority
    source_uav_id: str | None = None
    data: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.sort_key = (-self.priority.value, self.timestamp)
```

### 5.8 Assignment

```python
@dataclass
class Assignment:
    task_id: str
    uav_id: str
    bid_value: float
    assigned_at: float
```

### 5.9 PathPlan

```python
@dataclass
class PathPlan:
    uav_id: str
    task_id: str | None
    start: Position
    goal: Position
    path: list[Position]
    cost: float
    valid: bool
    reason: str | None = None
    planned_at: float = 0.0
    latency_ms: float = 0.0
```

### 5.10 Conflict

```python
@dataclass
class Conflict:
    uav_id_a: str
    uav_id_b: str
    time: float
    position_a: Position
    position_b: Position
    distance_cells: float
    severity: EventPriority
```

### 5.11 DecisionCommand 与 DecisionOutput

```python
@dataclass
class DecisionCommand:
    uav_id: str
    command: CommandType
    task_id: str | None
    target: Position | None
    path: list[Position]
    reason: str | None = None

@dataclass
class DecisionOutput:
    timestamp: float
    commands: list[DecisionCommand]
    assignments: list[Assignment]
    events_handled: list[str]
    global_coverage: float
    priority_coverage: float
    decision_latency_ms: float
```

## 6. 输入输出协议格式

虽然第一版系统采用单进程 Python 方法调用，但仍应定义外部 JSON 协议，作为未来接入 UI、服务接口或真实系统的稳定边界。

### 6.1 无人机状态输入

```json
{
  "timestamp": 120.0,
  "uavs": [
    {
      "id": "uav_01",
      "position": {"x": 12, "y": 35},
      "velocity_mps": 10.0,
      "heading_deg": 90.0,
      "battery": 0.76,
      "sensor_radius_cells": 2,
      "status": "SEARCHING",
      "current_task_id": "task_003",
      "available": true
    }
  ]
}
```

### 6.2 地图更新输入

```json
{
  "timestamp": 90.0,
  "type": "MAP_UPDATE",
  "updates": [
    {
      "operation": "SET_REGION",
      "region_type": "rectangle",
      "cell_type": "OBSTACLE",
      "points": [{"x": 40, "y": 50}, {"x": 45, "y": 56}]
    }
  ]
}
```

约定：

- `points` 使用栅格坐标。
- 若来自场景 YAML 的世界坐标，由加载器先转换为栅格坐标。
- `operation` 第一版支持 `SET_CELL`、`SET_REGION`、`CLEAR_REGION`。

### 6.3 目标发现事件输入

```json
{
  "id": "event_00042",
  "type": "TARGET_FOUND",
  "timestamp": 88.0,
  "priority": "CRITICAL",
  "source_uav_id": "uav_03",
  "data": {
    "target_id": "target_001",
    "position": {"x": 55, "y": 71},
    "confidence": 0.83,
    "target_type": "person"
  }
}
```

### 6.4 决策输出

```json
{
  "timestamp": 121.0,
  "commands": [
    {
      "uav_id": "uav_01",
      "command": "FOLLOW_PATH",
      "task_id": "task_003",
      "target": {"x": 20, "y": 40},
      "path": [{"x": 13, "y": 36}, {"x": 14, "y": 37}],
      "reason": "regular_cycle"
    }
  ],
  "assignments": [
    {
      "task_id": "task_003",
      "uav_id": "uav_01",
      "bid_value": 42.7,
      "assigned_at": 121.0
    }
  ],
  "progress": {
    "global_coverage": 0.42,
    "priority_coverage": 0.68
  },
  "decision_latency_ms": 38.5
}
```

## 7. 模块接口契约

### 7.1 地图模块

| 方法 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `load_map(config)` | dict | `GridMap` | 从配置生成地图 |
| `update_cell(pos, attrs)` | `Position`, dict | None | 更新单格 |
| `update_region(cells, attrs)` | `Iterable[Position]`, dict | list[Position] | 返回受影响栅格 |
| `is_passable(pos)` | `Position` | bool | 路径规划基础接口 |
| `mark_covered(center, radius, timestamp)` | `Position`, int, float | list[Position] | 更新覆盖置信度 |

### 7.2 无人机模块

| 方法 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `get_all_states()` | None | list[`UAVState`] | 获取当前全量状态 |
| `get_available_uavs()` | None | list[`UAVState`] | 返回可参与拍卖的 UAV |
| `assign_path(uav_id, path)` | str, list[`Position`] | None | 写入路径 |
| `set_status(uav_id, status)` | str, `UAVStatus` | None | 状态切换 |
| `release_task(uav_id)` | str | str \| None | 回收当前任务 ID |

### 7.3 任务模块

| 方法 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `generate_initial_tasks(grid_map, uav_count)` | `GridMap`, int | list[`Task`] | 初始化搜索任务 |
| `get_pending_tasks()` | None | list[`Task`] | 待分配任务 |
| `assign_task(task_id, uav_id)` | str, str | `Assignment` | 标记任务归属 |
| `complete_task(task_id)` | str | None | 完成任务 |
| `mark_blocked(task_id, reason)` | str, str | None | 标记不可达或失败 |
| `requeue_task(task_id)` | str | None | 任务回收到 pending |

### 7.4 分配模块

| 方法 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `calculate_bid(uav, task, grid_map)` | `UAVState`, `Task`, `GridMap` | float \| None | None 表示不参与竞标 |
| `allocate(tasks, uavs, grid_map, now)` | list[`Task`], list[`UAVState`], `GridMap`, float | list[`Assignment`] | 顺序单物品拍卖 |

### 7.5 路径规划模块

| 方法 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `astar_search(grid_map, start, goal)` | `GridMap`, `Position`, `Position` | list[`Position`] \| None | A* 路径 |
| `plan_path(uav, target, task_id, grid_map)` | `UAVState`, `Position`, str \| None, `GridMap` | `PathPlan` | 规划并记录耗时 |
| `is_path_valid(path, grid_map)` | list[`Position`], `GridMap` | bool | 动态地图更新后校验 |
| `detect_conflicts(fleet, horizon)` | list[`UAVState`], int | list[`Conflict`] | 时间展开冲突检测 |
| `resolve_conflicts(conflicts, fleet, grid_map)` | list[`Conflict`], `FleetManager`, `GridMap` | list[`DecisionCommand`] | 生成等待或重规划命令 |

### 7.6 调度器模块

| 方法 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `step(now)` | float | `DecisionOutput` | 执行一个调度周期 |
| `handle_event(event)` | `Event` | list[`DecisionCommand`] | 处理单个事件 |
| `handle_urgent_events(events)` | list[`Event`] | list[`DecisionCommand`] | 高优先级即时响应 |
| `regular_cycle(now)` | float | `DecisionOutput` | 固定周期任务更新、分配、规划 |

## 8. 状态机

### 8.1 UAV 状态转移

| 当前状态 | 触发条件 | 新状态 | 副作用 |
|----------|----------|--------|--------|
| `IDLE` | 分配搜索任务 | `SEARCHING` | 写入任务 ID 和路径 |
| `IDLE` | 电量低于阈值 | `RETURNING` | 规划返航路径 |
| `SEARCHING` | 发现目标 | `CONFIRMING` | 暂停或回收当前任务，生成确认任务 |
| `SEARCHING` | 任务完成 | `IDLE` | 标记任务完成，清空路径 |
| `SEARCHING` | 电量低于阈值 | `RETURNING` | 回收任务，规划返航 |
| `SEARCHING` | 检测到局部冲突 | `AVOIDING` | 等待或重规划 |
| `CONFIRMING` | 确认完成 | `IDLE` | 记录目标确认，参与下一轮拍卖 |
| `CONFIRMING` | 电量低于阈值 | `RETURNING` | 记录未完成确认，规划返航 |
| `AVOIDING` | 冲突解除 | 原任务状态 | 继续执行原任务 |
| `RETURNING` | 到达 home | `IDLE` | 标记可用或结束任务 |
| 任意 | UAV 离线 | `OFFLINE` | `available=False`，回收任务 |
| `OFFLINE` | UAV 恢复 | `IDLE` | `available=True`，等待拍卖 |

优先级：

1. 离线事件优先级最高。
2. 低电量优先于普通搜索任务，但低电量策略不得打断已经无法安全返回的路径校验。
3. 目标确认优先于普通搜索。

### 8.2 Task 状态转移

| 当前状态 | 触发条件 | 新状态 |
|----------|----------|--------|
| `PENDING` | 拍卖分配成功 | `ASSIGNED` |
| `ASSIGNED` | UAV 开始执行 | `IN_PROGRESS` |
| `IN_PROGRESS` | 覆盖率或确认条件满足 | `COMPLETED` |
| `IN_PROGRESS` | UAV 离线或低电量返航 | `PENDING` |
| `PENDING` / `ASSIGNED` | 目标不可达或区域不可达 | `BLOCKED` |
| 任意 | 人工取消或场景终止 | `CANCELLED` |
| `BLOCKED` | 地图更新后恢复可达 | `PENDING` |

### 8.3 Event 处理优先级

| 事件 | 默认优先级 | 处理时机 |
|------|------------|----------|
| `TARGET_FOUND` | `CRITICAL` | 立即 |
| `UAV_OFFLINE` | `CRITICAL` | 立即 |
| `LOW_BATTERY` | `HIGH` | 立即 |
| `MAP_UPDATE` | `HIGH` | 防抖合并后尽快 |
| `CONFIRM_DONE` | `NORMAL` | 当前或下一周期 |
| `TASK_BLOCKED` | `NORMAL` | 下一周期 |
| `CONFLICT_DETECTED` | `HIGH` | 当前周期内 |

## 9. 配置文件 Schema

### 9.1 默认配置

```yaml
map:
  width_m: 1000
  height_m: 1000
  resolution_m: 10

uav:
  count: 6
  max_speed_mps: 10.0
  sensor_radius_cells: 2
  endurance_s: 1800
  battery_threshold: 0.2
  home_position: [0, 0]

search:
  decay_lambda: 0.01
  decay_threshold: 0.3
  coverage_complete_threshold: 0.95
  confirm_duration_steps: 5

auction:
  w_distance: 1.0
  w_battery: 0.3
  w_priority: 0.5
  w_balance: 0.2
  use_astar_for_bid: false

planning:
  obstacle_proximity_penalty: 0.5
  priority_area_bonus: -0.2
  safety_distance_cells: 2
  conflict_time_horizon_steps: 60
  max_astar_time_ms: 100

scheduler:
  cycle_interval_s: 1.0
  event_debounce_s: 0.2
  max_decision_latency_ms: 1000

simulation:
  time_step_s: 1.0
  max_steps: 2000
  random_seed: 42

visualization:
  fps: 10
  show_paths: true
  show_coverage: true
  show_grid: false

logging:
  output_dir: "runs"
  snapshot_interval_steps: 1
  save_json: true
```

### 9.2 校验规则

| 字段 | 规则 |
|------|------|
| `map.width_m`, `map.height_m` | 必须大于 0 |
| `map.resolution_m` | 必须大于 0，且宽高应能被分辨率合理划分 |
| `uav.count` | 建议范围 1-10 |
| `uav.battery_threshold` | `[0.0, 1.0]` |
| `search.decay_lambda` | `>= 0` |
| `planning.safety_distance_cells` | `>= 0` |
| `scheduler.cycle_interval_s` | `> 0` |
| `simulation.time_step_s` | `> 0` |

## 10. 场景文件 Schema

场景配置用于覆盖默认配置，并定义障碍物、禁飞区、重点区域、初始 UAV 和预设事件。

```yaml
name: "dynamic_city_search"
description: "动态城市搜索场景"

overrides:
  uav:
    count: 6
  simulation:
    random_seed: 42

map_features:
  obstacles:
    - id: "building_001"
      shape: "rectangle"
      frame: "world"
      x_m: 100
      y_m: 200
      width_m: 80
      height_m: 120

  no_fly_zones:
    - id: "school_zone"
      shape: "polygon"
      frame: "world"
      points_m:
        - [300, 300]
        - [420, 300]
        - [420, 440]
        - [300, 440]

  priority_zones:
    - id: "incident_area"
      shape: "rectangle"
      frame: "world"
      x_m: 600
      y_m: 200
      width_m: 150
      height_m: 150
      priority: 3.0

uavs:
  - id: "uav_01"
    home_position: [0, 0]
    initial_position: [0, 0]
    battery: 1.0

events:
  - time_s: 60
    type: "TARGET_APPEAR"
    data:
      target_id: "target_001"
      position: [55, 70]
      target_type: "person"

  - time_s: 90
    type: "MAP_UPDATE"
    data:
      operation: "SET_REGION"
      cell_type: "OBSTACLE"
      shape: "rectangle"
      frame: "world"
      x_m: 400
      y_m: 500
      width_m: 50
      height_m: 80
```

约定：

- `frame: world` 表示坐标单位为米。
- `frame: grid` 表示坐标单位为栅格。
- 如果场景中未显式给出 UAV 列表，则根据默认配置自动生成。

## 11. 错误处理策略

| 失败场景 | 处理策略 | 记录 |
|----------|----------|------|
| A* 找不到路径 | 返回 `PathPlan(valid=False)`，任务标记 `BLOCKED` 或换 entry point 重试 | `TASK_BLOCKED` |
| 拍卖时无 UAV 可用 | 任务保持 `PENDING`，下一周期重试 | scheduler log |
| UAV 电量不足以执行任务并返航 | 该 UAV 不参与该任务竞标 | bid log |
| 低电量但返航路径不可达 | 标记 `CRITICAL`，命令 `HOLD`，记录失败 | `LOW_BATTERY` + error |
| 动态障碍物阻断当前路径 | 局部重规划；失败则任务回收 | `MAP_UPDATE` |
| 冲突无法通过等待消解 | 低优先级 UAV 重规划；仍失败则 `HOLD` | `CONFLICT_DETECTED` |
| 子区域划分不连通 | BFS 修复；仍失败则拆分任务 | task generation log |
| 覆盖路径为空 | 任务标记 `BLOCKED`，不进入拍卖 | task log |
| 配置非法 | 启动失败并输出具体字段 | config validation error |

原则：

- 算法失败必须返回可解释对象，不允许静默失败。
- `None` 仅用于内部简短失败信号，跨模块输出应使用带 `valid/reason` 的结构。
- 所有会影响任务状态的失败都必须写入事件或日志。

## 12. 日志与实验结果格式

每次仿真运行生成一个 run 目录：

```text
runs/
  scenario_3_seed_42_20260611_021500/
    config.resolved.yaml
    snapshots.json
    events.json
    metrics.json
    coverage.png
    trajectories.png
```

### 12.1 Snapshot JSON

```json
{
  "run_id": "scenario_3_seed_42",
  "steps": [
    {
      "time_s": 1.0,
      "global_coverage": 0.03,
      "priority_coverage": 0.0,
      "uavs": [
        {
          "id": "uav_01",
          "position": {"x": 1, "y": 0},
          "status": "SEARCHING",
          "battery": 0.998,
          "task_id": "task_001"
        }
      ],
      "events": []
    }
  ]
}
```

### 12.2 Metrics JSON

```json
{
  "run_id": "scenario_3_seed_42",
  "global_coverage": 0.96,
  "priority_coverage": 1.0,
  "time_to_95_coverage_s": 842.0,
  "redundant_coverage_rate": 0.12,
  "allocation_balance_std": 0.8,
  "conflict_count": 0,
  "no_fly_violations": 0,
  "min_battery_margin": 0.23,
  "decision_latency_avg_ms": 42.5,
  "decision_latency_p95_ms": 88.1,
  "replan_latency_avg_ms": 55.2
}
```

## 13. 测试矩阵

### 13.1 单元测试

| 模块 | 用例 | 期望 |
|------|------|------|
| 地图 | 世界坐标转栅格坐标 | 与分辨率规则一致 |
| 地图 | 障碍物栅格化 | 对应栅格 `passable=False` |
| 地图 | 禁飞区优先级覆盖重点区 | 重叠区域不可通行 |
| 地图 | `get_neighbors` 8 邻域 | 不返回越界或不可通行栅格 |
| UAV | 电量消耗 | 随距离增加而下降 |
| UAV | 低电量检测 | 低于阈值触发返回状态 |
| 任务 | 子区域划分数量 | 输出 N 个区域 |
| 任务 | 子区域连通性 | 每个区域 BFS 连通 |
| 任务 | 覆盖路径生成 | waypoint 非空且不进入不可通行格 |
| A* | 开阔地图最短路径 | 路径长度符合预期 |
| A* | 障碍物绕行 | 不穿越障碍物 |
| A* | 不可达目标 | 返回 invalid 或 None |
| 拍卖 | 优先级排序 | 高优先级任务先分配 |
| 拍卖 | 电量不足过滤 | UAV 不参与竞标 |
| 冲突 | 同格同刻 | 检测出冲突 |
| 冲突 | 等待消解 | 消解后无冲突 |

### 13.2 集成测试

| 阶段 | 用例 | 期望 |
|------|------|------|
| 阶段一 | 简单地图 + 单 UAV 预设路径 | 可视化移动，覆盖率更新 |
| 阶段二 | 单 UAV 覆盖子区域 | 覆盖率接近 100% |
| 阶段三 | 障碍地图路径规划 | UAV 绕障，无禁飞区侵入 |
| 阶段四 | 5 架 UAV 协同搜索 | 任务分配均衡，冲突数为 0 |
| 阶段五 | t=60 出现目标 | 发现者 1 周期内转 CONFIRMING |
| 阶段五 | t=90 新障碍物阻断路径 | 受影响 UAV 重规划 |
| 阶段五 | UAV 离线 | 任务回收并重新分配 |
| 阶段六 | 动态场景完整运行 | 输出 metrics 和报告图 |

### 13.3 性能测试

| 指标 | 目标 |
|------|------|
| 200x200 栅格单次 A* | `< 100ms` |
| 单步调度周期 | `< 1000ms` |
| 5-10 架 UAV 冲突检测 | `< 200ms` |
| 场景 3 完整仿真 | 不出现死循环或调度停滞 |

## 14. 开发顺序建议

建议将正式开发拆成更细的起步顺序：

1. 创建工程骨架、配置加载、数据结构和枚举。
2. 实现 `GridMap`、坐标转换、障碍/禁飞/重点区域加载。
3. 实现 `UAVState`、`FleetManager` 和最小仿真步进。
4. 实现 A* 最小版本，并接入单 UAV 路径执行。
5. 实现任务模型和单区域覆盖路径。
6. 实现任务管理和拍卖分配。
7. 实现冲突检测与消解。
8. 实现事件管理、动态重规划和低电量处理。
9. 实现指标、日志和报告。

原因：先让“地图 - UAV - 路径 - 仿真”形成最短闭环，再逐步加入任务、拍卖和事件，能最大限度降低集成风险。

## 15. 首轮开发 Definition of Done

首轮开发完成不以“所有功能写完”为标准，而以以下闭环为准：

- 能从 YAML 加载一个 1000m x 1000m、10m 分辨率的场景。
- 能初始化至少 1 架 UAV。
- 能在含障碍物的地图上规划从起点到目标点的 A* 路径。
- UAV 能沿路径移动并更新覆盖置信度。
- 能输出 `DecisionOutput` 风格的命令对象。
- 能保存基础 `snapshots.json`。
- 至少包含地图、UAV、A* 三类单元测试。

达到该标准后，再进入多任务、多机协同和动态事件开发。
