# 多无人机区域协同搜索决策模型 — 详细实现计划

基于设计方案 `2026-06-11-multi-uav-search-design.md`，本文档将 6 个开发阶段细化为可执行的开发任务。

---

## 阶段一：基础框架（第 1 周）

### 目标
搭建项目骨架，实现栅格地图和无人机状态模型，跑通基本仿真循环。

### 任务清单

#### 1.1 项目初始化
- 创建工程目录结构（按设计文档第十三节）
- 编写 `requirements.txt`（numpy, matplotlib, pyyaml, pytest）
- 编写 `config/default.yaml` 基础配置模板
- 编写 `README.md` 项目说明

#### 1.2 公共数据结构 (`core/data_types.py`)
- 定义 `Position` 数据类（x, y 栅格坐标）
- 定义 `CellType` 枚举（FREE, OBSTACLE, NO_FLY, PRIORITY）
- 定义 `UAVStatus` 枚举（IDLE, SEARCHING, CONFIRMING, RETURNING, AVOIDING）
- 定义 `EventType` 枚举（TARGET_FOUND, UAV_OFFLINE, MAP_UPDATE, LOW_BATTERY, CONFIRM_DONE）
- 定义 `Event` 数据类（type, timestamp, data, priority）

#### 1.3 栅格地图模块 (`map/`)
- `grid_map.py`：实现 `GridMap` 类
  - 构造函数接收宽、高、分辨率
  - 内部使用 numpy 二维数组存储各属性层（terrain, passable, search_confidence, priority, last_search_time）
  - 提供方法：`is_passable(x, y)`, `get_cell(x, y)`, `set_cell(x, y, attrs)`, `get_neighbors(x, y, mode=8)`
  - 提供批量查询：`get_unsearched_cells()`, `get_priority_cells()`
- `map_loader.py`：实现从 YAML 配置加载地图
  - 支持定义矩形障碍物区域、禁飞区多边形、重点区域
  - 将几何描述转化为栅格标记
- `map_updater.py`：实现动态更新接口
  - `update_cell(x, y, attrs)`：单栅格更新
  - `update_region(cells, attrs)`：批量更新
  - 更新后记录受影响区域，供路径模块查询

#### 1.4 无人机模型 (`uav/`)
- `uav_model.py`：实现 `UAV` 类
  - 属性：id, position, velocity, heading, battery, sensor_radius, status, current_task, path
  - 方法：`move_to(next_position)`, `consume_battery(distance)`, `can_reach_and_return(target, home)`
  - 电量消耗计算逻辑
- `fleet_manager.py`：实现 `FleetManager` 类
  - 管理所有 UAV 实例
  - 提供 `get_all_states()`, `get_available_uavs()`, `get_uav(id)`

#### 1.5 基础仿真循环 (`simulation/simulator.py`)
- 实现 `Simulator` 类
  - 时间步进：每步推进一个时间单位（默认 1 秒）
  - 无人机运动模拟：按速度沿路径移动，更新位置
  - 传感器覆盖模拟：无人机飞过时更新周围栅格的搜索置信度
- 实现基本主循环 (`main.py`)：加载地图 → 初始化无人机 → 循环步进

#### 1.6 基础可视化 (`visualization/realtime_viewer.py`)
- 使用 matplotlib 绘制栅格地图（不同颜色表示不同地形）
- 绘制无人机位置（带方向箭头）
- 实现逐帧动画更新（`FuncAnimation`）

#### 1.7 单元测试
- `tests/test_map.py`：测试地图加载、栅格查询、动态更新
- 测试无人机电量计算、位置移动

### 验收标准
- 运行 `main.py` 可看到地图可视化，无人机在地图上按预设路径移动
- 所有单元测试通过

---

## 阶段二：搜索任务（第 2 周）

### 目标
实现区域划分和任务生成，单架无人机可执行覆盖搜索。

### 任务清单

#### 2.1 区域划分算法 (`task/task_generator.py`)
- 实现基于 K-Means 的区域划分
  - 输入：可搜索栅格坐标集合、无人机数量 N
  - 输出：N 个子区域（栅格坐标集合列表）
  - 约束：每个子区域连通、面积尽量均衡
- 连通性修复：K-Means 结果可能不连通，通过 BFS 将孤立块合并到最近子区域
- 生成每个子区域的建议进入点（离无人机起飞点最近的边界栅格）

#### 2.2 覆盖路径生成 (`task/task_generator.py`)
- 实现 Boustrophedon（往返覆盖）路径生成
  - 输入：子区域栅格集合、传感器覆盖半径
  - 输出：航路点序列
  - 相邻航线间距 = 2 × sensor_radius
- 处理不规则形状子区域：按行扫描，跳过障碍物栅格

#### 2.3 任务数据结构 (`task/task_model.py`)
- 实现 `Task` 数据类
  - 字段：id, type(SEARCH/CONFIRM), priority, target_cells, entry_point, estimated_cost, status, assigned_uav, waypoints
- 实现 `TaskStatus` 枚举：PENDING, ASSIGNED, IN_PROGRESS, COMPLETED

#### 2.4 任务管理器 (`task/task_manager.py`)
- 实现 `TaskManager` 类
  - 启动时调用 task_generator 生成初始任务集
  - 维护任务池：pending_tasks, active_tasks, completed_tasks
  - 提供方法：`get_pending_tasks()`, `assign_task(task_id, uav_id)`, `complete_task(task_id)`, `update_progress()`
  - 搜索置信度衰减检测：已完成任务若衰减到阈值以下，重新加入 pending

#### 2.5 搜索衰减机制 (`map/grid_map.py` 扩展)
- 实现 `decay_search_confidence(current_time, lambda_decay)` 方法
- 每个决策周期调用，更新所有已搜索栅格的置信度

#### 2.6 集成测试
- 单架无人机从起飞点出发，按覆盖路径完成一个子区域搜索
- 验证搜索后栅格置信度更新正确
- 验证搜索衰减机制工作正常

### 验收标准
- 可视化展示：区域被划分为 N 个彩色子区域，单架无人机按往返路径覆盖其分配区域
- 搜索覆盖率随时间增长直到接近 100%

---

## 阶段三：路径规划（第 3 周）

### 目标
实现 A* 路径规划、路径平滑，无人机可规划并执行绕障路径。

### 任务清单

#### 3.1 A* 算法核心 (`planning/astar.py`)
- 实现 `astar_search(grid_map, start, goal)` 函数
  - 8 邻域移动，直线代价 1.0，对角代价 1.414
  - 启发函数：对角距离（Diagonal Distance）
  - 返回路径点列表或 None（不可达）
- 实现代价增强：
  - `obstacle_proximity_penalty`：距障碍物 1 格内的栅格增加额外代价（默认 +0.5）
  - `priority_area_bonus`：未搜索重点区域栅格减少代价（默认 -0.2）
- 性能验证：200×200 地图上单次规划 < 100ms

#### 3.2 路径后处理 (`planning/path_smoother.py`)
- 实现路径简化：移除共线中间点（基于方向向量比较）
- 实现拐角平滑：对锐角转弯插入中间过渡点
- 实现安全校验：平滑后的路径逐段检查是否穿越不可通行栅格

#### 3.3 路径规划器封装 (`planning/` 顶层)
- 实现 `PathPlanner` 类
  - `plan_path(uav, target, grid_map)` → 调用 A* + 后处理
  - `is_path_valid(path, grid_map)` → 检查路径是否仍然可通行
  - `replan_path(uav, grid_map)` → 重新规划当前目标的路径

#### 3.4 与仿真循环集成
- 无人机按 A* 规划路径移动（替代阶段一的预设路径）
- 遇到动态新增障碍物时触发重规划

#### 3.5 单元测试
- `tests/test_astar.py`：
  - 开阔地图最短路径验证
  - 有障碍物的绕行路径验证
  - 不可达目标返回 None
  - 路径平滑不穿越障碍物
  - 性能基准测试（200×200 地图 < 100ms）

### 验收标准
- 可视化展示：无人机在有障碍物的地图上规划路径并平滑移动
- A* 性能满足 < 100ms
- 路径不穿越障碍物或禁飞区

---

## 阶段四：多机协同（第 4–5 周，1.5 周）

### 目标
实现拍卖任务分配和冲突检测，多架无人机协同搜索不冲突。

### 任务清单

#### 4.1 竞标值计算 (`allocation/bid_calculator.py`)
- 实现 `calculate_bid(uav, task, grid_map, fleet)` 函数
  - distance_cost：A* 规划路径长度（或曼哈顿距离近似，避免拍卖时大量 A* 调用）
  - battery_cost：预估耗电 / 剩余电量
  - priority_bonus：task.priority 的负值加权
  - load_balance_penalty：该无人机已分配任务数 × 惩罚系数
- 权重 w1–w4 从配置文件读取，默认 w1=1.0, w2=0.3, w3=0.5, w4=0.2

#### 4.2 拍卖算法 (`allocation/auction.py`)
- 实现 `SequentialAuction` 类
  - `allocate(pending_tasks, available_uavs, grid_map)` 方法
  - 流程：
    1. 任务按优先级降序排列
    2. 对每个任务，所有可用无人机计算 bid
    3. 选 bid 最低者分配
    4. 更新该无人机状态，进入下一任务
  - 特殊处理：CONFIRM 任务直接指派，不走拍卖
  - 电量过滤：无法完成任务的无人机不参与

#### 4.3 冲突检测 (`planning/conflict_resolver.py`)
- 实现 `detect_conflicts(fleet, time_horizon)` 函数
  - 将每架无人机路径按时间步展开为 (position, time) 序列
  - 两两比对，检查同一时刻是否距离 < safety_distance（默认 2 格）
  - 返回冲突列表：[(uav_i, uav_j, time, position)]

#### 4.4 冲突消解 (`planning/conflict_resolver.py`)
- 实现 `resolve_conflicts(conflicts, fleet, grid_map)` 函数
  - 按冲突时间排序处理
  - 策略 1（时间偏移）：低优先级无人机在冲突点前等待 1–2 步
  - 策略 2（路径偏移）：将高优先级无人机路径临时标记为障碍，为低优先级无人机重新规划
  - 优先级规则：CONFIRMING > SEARCHING > RETURNING > IDLE；同状态电量低者优先

#### 4.5 调度器集成 (`core/scheduler.py`)
- 实现 `Scheduler` 类，整合主循环：
  - 固定周期调用：任务更新 → 拍卖分配 → 路径规划 → 冲突检测与消解
  - 将之前 main.py 中的临时循环替换为 Scheduler 驱动

#### 4.6 多机仿真验证
- 配置 5–8 架无人机的场景
- 验证任务分配均衡性
- 验证路径无冲突

#### 4.7 单元测试
- `tests/test_auction.py`：拍卖分配正确性、电量过滤、优先级排序
- `tests/test_conflict.py`：冲突检测准确性、消解后路径无冲突

### 验收标准
- 5–8 架无人机协同搜索，各负责不同子区域
- 全程无空间冲突（冲突次数 = 0）
- 任务分配基本均衡（标准差 < 均值的 30%）

---

## 阶段五：动态响应（第 5–6 周）

### 目标
实现事件驱动重规划、目标发现响应和电量管理。

### 任务清单

#### 5.1 事件管理器 (`core/event_manager.py`)
- 实现 `EventManager` 类
  - 事件队列（优先级队列）
  - `emit(event)` 发送事件
  - `poll_events()` 获取当前周期所有待处理事件
  - `register_handler(event_type, callback)` 注册监听
  - 防抖机制：200ms 内同类事件合并

#### 5.2 目标发现与确认流程
- 仿真中随机或预设时刻在指定栅格触发目标出现
- 无人机传感器覆盖到目标栅格时触发 TARGET_FOUND 事件
- 事件处理：
  1. 发现者状态切换为 CONFIRMING
  2. 生成 CONFIRM 任务（目标位置）
  3. 发现者路径重规划至目标位置
  4. 确认完成后（到达目标位置并停留 N 步），发送 CONFIRM_DONE 事件
  5. 发现者恢复 IDLE，其原搜索任务如未完成则重新入池

#### 5.3 电量管理与强制返航
- 每步检查所有无人机电量
- 电量 < 阈值（20%）时：
  1. 发送 LOW_BATTERY 事件
  2. 无人机状态切换为 RETURNING
  3. 路径重规划至起飞点
  4. 其未完成任务回收到 pending 池，下轮拍卖重新分配

#### 5.4 无人机离线模拟
- 支持仿真中手动或随机触发无人机离线
- 离线处理：
  1. 发送 UAV_OFFLINE 事件
  2. 该无人机从 fleet 中标记为不可用
  3. 其所有任务回收重分配

#### 5.5 地图动态更新响应
- 仿真中模拟新障碍物出现
- MAP_UPDATE 事件触发后：
  1. 更新栅格地图
  2. 检查所有无人机路径是否受影响（`is_path_valid`）
  3. 受影响无人机重新规划路径
  4. 若障碍物阻断了某任务区域的可达性，任务重新生成

#### 5.6 重规划调度逻辑 (`core/scheduler.py` 扩展)
- 高优先级事件（TARGET_FOUND, UAV_OFFLINE, LOW_BATTERY）立即处理，不等周期
- 常规事件在下一个固定周期统一处理
- 实现防抖：连续多个 MAP_UPDATE 合并为一次批量重规划

#### 5.7 集成测试场景
- 场景 A：搜索过程中 t=60s 出现目标，验证发现者响应流程
- 场景 B：某无人机 t=120s 电量耗尽，验证任务回收与重分配
- 场景 C：t=90s 出现新障碍物阻断路径，验证动态重规划

### 验收标准
- 目标发现后，发现者在 1 个决策周期内开始抵近
- 低电量无人机安全返航，任务被其他无人机接管
- 新障碍物出现后受影响路径自动更新，无无人机撞入障碍物

---

## 阶段六：评估优化（第 6–7 周）

### 目标
实现评价指标体系，进行多场景实验，输出分析报告。

### 任务清单

#### 6.1 评价指标实现 (`evaluation/metrics.py`)
- 覆盖率指标：
  - `global_coverage(grid_map)` → float
  - `priority_coverage(grid_map)` → float
  - `coverage_uniformity(grid_map, regions)` → float (标准差)
- 时间效率指标：
  - `time_to_coverage(history, target_rate=0.95)` → int (步数)
  - `priority_first_coverage_time(history)` → int
  - `target_discovery_delay(targets)` → float (平均延迟)
- 协同效率指标：
  - `redundant_coverage_rate(grid_map)` → float
  - `allocation_balance(fleet)` → float (标准差)
  - `path_efficiency(fleet)` → float (有效搜索路程/总路程)
- 安全性指标：
  - `conflict_count(log)` → int
  - `no_fly_violations(log)` → int
  - `min_battery_margin(fleet)` → float
- 动态响应指标：
  - `replan_latency(log)` → float (平均毫秒)
  - `confirm_response_time(log)` → float
  - `task_recovery_time(log)` → float

#### 6.2 数据记录器 (`simulation/simulator.py` 扩展)
- 仿真过程中记录每步状态快照：
  - 所有无人机位置、状态、电量
  - 地图搜索覆盖率
  - 事件日志（时间戳、类型、详情）
- 保存为 JSON 或 CSV 文件，支持离线分析

#### 6.3 报告生成 (`visualization/report_generator.py`)
- 覆盖率随时间变化曲线
- 各无人机飞行轨迹图
- 任务分配统计柱状图
- 指标对比表格（不同配置/场景间）
- 输出为 matplotlib 图片或汇总 PDF

#### 6.4 多场景配置
- `config/scenarios/scenario_1.yaml`：基础场景（少量障碍物，无目标出现）
- `config/scenarios/scenario_2.yaml`：复杂场景（多障碍物，禁飞区，重点区域）
- `config/scenarios/scenario_3.yaml`：动态场景（运行中出现目标、新障碍物、无人机离线）
- `config/scenarios/scenario_4.yaml`：压力测试（10 架无人机，2km×2km 区域）

#### 6.5 参数敏感性实验
- 变量：栅格分辨率（5m, 10m, 20m）
- 变量：无人机数量（3, 5, 8, 10）
- 变量：拍卖权重配置（距离主导 vs 均衡配置）
- 变量：衰减系数 λ（0, 0.01, 0.05）
- 每组实验运行 3 次取平均

#### 6.6 性能优化（按需）
- 若 A* 耗时超标：实现 JPS 或局部搜索
- 若拍卖耗时超标：使用曼哈顿距离近似替代完整 A* 估价
- 若冲突检测耗时超标：空间索引加速（网格划分）

#### 6.7 最终集成测试
- 完整跑通 scenario_3（动态场景），全部指标达标
- 生成完整实验报告

### 验收标准
- 4 个场景全部可运行并输出指标
- 生成对比分析图表
- 安全性指标全部达标（冲突=0，禁飞侵入=0）
- 单步决策延迟 < 1 秒

---

## 配置文件模板

### `config/default.yaml`

```yaml
# 地图配置
map:
  width: 1000          # 区域宽度 (m)
  height: 1000         # 区域高度 (m)
  resolution: 10       # 栅格分辨率 (m)

# 无人机配置
uav:
  count: 6
  max_speed: 10        # m/s
  sensor_radius: 2     # 栅格数
  endurance: 1800      # 续航时间 (s)
  battery_threshold: 0.2
  home_position: [0, 0]

# 搜索配置
search:
  decay_lambda: 0.01
  decay_threshold: 0.3
  confirm_duration: 5  # 确认停留步数

# 拍卖配置
auction:
  w_distance: 1.0
  w_battery: 0.3
  w_priority: 0.5
  w_balance: 0.2

# 路径规划配置
planning:
  obstacle_proximity_penalty: 0.5
  priority_area_bonus: -0.2
  safety_distance: 2   # 冲突检测安全距离 (栅格)

# 调度配置
scheduler:
  cycle_interval: 1.0  # 固定周期 (s)
  event_debounce: 0.2  # 事件防抖窗口 (s)

# 仿真配置
simulation:
  time_step: 1.0       # 仿真步长 (s)
  max_steps: 2000      # 最大仿真步数
  random_seed: 42

# 可视化配置
visualization:
  fps: 10
  show_paths: true
  show_coverage: true
  show_grid: false
```

---

## 开发规范

### 代码风格
- Python 3.10+，使用 type hints
- 使用 `dataclass` 定义所有数据结构
- 模块间通过方法调用通信，不使用全局变量
- 配置通过 YAML 文件加载，不硬编码

### 测试策略
- 每个模块附带单元测试
- 阶段结束时进行集成测试
- 使用 pytest 运行，目标覆盖率 > 80%

### 版本管理
- 每个阶段完成后打 tag（v0.1 ~ v0.6）
- 功能开发在 feature 分支进行，完成后合并到 main

---

## 依赖与风险

| 风险项 | 影响 | 应对策略 |
|--------|------|----------|
| A* 在大地图上性能不足 | 决策超时 | 预留 JPS 替换方案，或限制搜索范围 |
| K-Means 划分结果不连通 | 任务不可达 | 加入连通性修复后处理 |
| 拍卖分配局部最优 | 覆盖效率低 | 参数调优 + 多轮实验对比 |
| matplotlib 动画卡顿 | 可视化不流畅 | 降低刷新率或按需绘制 |
| 冲突消解死锁 | 两架无人机互相让路 | 引入严格优先级序，保证消解有终止 |
