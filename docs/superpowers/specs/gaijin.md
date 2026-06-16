请基于当前 `zhencha` 项目实现一版“收益驱动的多无人机协同搜索策略”。不要大规模重写项目架构，优先在现有 `Scheduler`、`TaskGenerator`、`Auction`、`TaskManager`、`Metrics` 上做增量改造。

## 总体目标

当前项目虽然能完成多 UAV 区域搜索、目标发现、盘旋确认、补搜和返航，但策略表现不够智能，主要问题是：

1. 后期为了零散小格子追求 100% 覆盖，导致 UAV 跨地图补碎片。
2. 空闲 UAV 有时等待或继续执行低价值任务，不能独立返航。
3. 初始任务分区没有充分考虑 UAV 起点、重点区域、障碍绕行成本和实际航线长度。
4. 拍卖分配过于依赖距离，重点区域优先级和任务收益影响太弱。
5. 任务入口点虽然被用于竞标，但实际执行航线没有根据 UAV 位置重排，可能从远端开始扫。
6. 发现目标后虽然能盘旋确认，但确认完成后的续搜策略仍然比较机械。

请实现一个“两阶段收益驱动策略”：

* 第一阶段：主搜索，快速覆盖高价值区域和大部分普通区域。
* 第二阶段：补搜，只处理值得搜索的残余区域，不强制追求 100% 普通覆盖。
* 重点区域必须更高覆盖率，普通零散碎片可以放弃。
* UAV 发现目标后盘旋确认，确认结束后重新参与收益驱动分配。

## 配置改造

在 `config/default.yaml` 的 `search` 中新增配置：

```yaml
search:
  mission_complete_coverage_threshold: 0.92
  priority_complete_threshold: 0.98
  min_supplemental_cells: 8
  min_supplemental_score: 0.15
  supplemental_cluster_max_cells: 80
  supplemental_cluster_radius_cells: 4
  allow_early_return: true
  priority_cell_weight: 3.0
  distance_cost_weight: 1.0
  uncovered_value_weight: 1.0
  priority_value_weight: 2.0
  redundant_penalty_weight: 0.5
```

保留已有 `coverage_complete_threshold`，它仍用于判断单格是否算已覆盖。

## 第一部分：补搜策略升级

保留 `gaijin.md` 的核心思路，但实现时要更完整。

修改 `Scheduler._ensure_supplemental_search_tasks()` 和相关辅助函数：

1. 不再直接把所有未覆盖格子变成补搜任务。
2. 获取未覆盖格子后，先排除：

   * 已经在 pending search task 中的格子；
   * 正在执行任务的 UAV 未来路径传感器覆盖区；
   * 已被其他补搜任务预订的格子。
3. 将剩余未覆盖格子聚类为候选簇。
4. 每个候选簇计算：

   * `uncovered_cells`
   * `priority_uncovered_cells`
   * `uncovered_value`
   * `priority_value`
   * `nearest_uav_distance`
   * `estimated_cost_m`
   * `score = value / max(cost, 1)`
5. 普通候选簇如果满足以下任一条件，则忽略：

   * 格子数小于 `min_supplemental_cells`
   * score 小于 `min_supplemental_score`
6. 重点区域例外：

   * 如果重点区域覆盖率低于 `priority_complete_threshold`，即使候选簇很小也要生成补搜任务。
7. 补搜任务排序：

   * score 降序
   * priority_value 降序
   * nearest_uav_distance 升序
   * cluster_size 降序

## 第二部分：任务完成与返航策略

修改当前返航逻辑。不要再要求全图 100% 覆盖后才返航。

新的任务完成条件：

```text
priority_coverage >= priority_complete_threshold
AND
(
  global_coverage >= mission_complete_coverage_threshold
  OR 当前不存在高价值补搜候选
)
```

当 UAV 变为 IDLE 时：

1. 如果还有高价值任务，则参与拍卖。
2. 如果没有高价值任务，并且任务完成条件满足，则立即返航。
3. 如果全局覆盖未达标，但该 UAV 到所有候选簇成本过高、score 不达标，也允许返航。
4. UAV 不应为了等待其他 UAV 而长时间 IDLE。

需要新增或修改函数：

* `_mission_goal_met()`
* `_priority_goal_met()`
* `_has_valuable_supplemental_candidates()`
* `_dispatch_idle_returns()`

## 第三部分：初始任务生成改造

不要只按连通区域均分。当前 `partition_search_area()` 太机械，需要改成更考虑任务价值和 UAV 起点。

要求：

1. 初始任务生成时考虑：

   * UAV 初始位置或 home position；
   * 重点区域 priority；
   * 障碍和禁飞区导致的绕行；
   * 每个任务预计航线长度；
   * 每个任务覆盖价值。
2. 每个初始任务必须计算并填写：

   * `estimated_cost_m`
   * `priority`
   * `entry_point`
   * `target_cells`
   * `waypoints`
3. `estimated_cost_m` 不能继续默认为 0，应至少包含：

   * 从最近 UAV/home 到入口点的距离估算；
   * 任务内部航线长度估算。
4. 优先保证任务规模和路径成本相对均衡，而不是只保证格子数量均衡。

如果不想一次性重写分区算法，可以先做保守改法：

* 保留现有连通区域切分；
* 但切分后为每个 region 计算真实任务价值和估算成本；
* 对重点区域所在 region 提高 priority；
* 对航线入口进行重排；
* 在拍卖中真正使用 estimated cost 和任务价值。

## 第四部分：航线入口与扫描方向重排

当前任务虽然有 `entry_point`，但实际执行时仍然按原始 `waypoints` 顺序飞，可能导致 UAV 绕到远端才开始搜索。

请新增一个函数，例如：

```python
reorder_waypoints_for_uav(waypoints, uav_position)
```

要求：

1. 找到距离当前 UAV 最近的可行航点作为入口。
2. 如果反转航线能降低起始距离，则反转。
3. 对 Boustrophedon 航线，尽量从离 UAV 最近的一端开始扫。
4. 任务分配后，在 `_plan_route_through_waypoints()` 前使用重排后的 waypoints。

补搜任务也必须使用这个逻辑。

## 第五部分：拍卖评分改造

当前拍卖主要看距离，priority 影响太弱。请修改 `calculate_bid()`，让任务价值真正影响分配。

建议评分：

```text
cost = travel_distance + estimated_task_cost + battery_penalty + load_balance_penalty
value = uncovered_value + priority_value
bid = cost / max(value, 1)
```

或者：

```text
bid = distance_cost
    + estimated_task_cost_weight * estimated_task_cost
    + battery_penalty
    + load_balance_penalty
    - value_bonus
```

要求：

1. 不能让 priority 只产生极小影响。
2. 重点区域任务应明显优先于普通区域。
3. 远距离低价值任务不应被分配。
4. 每轮每架 UAV 最多获得一个任务的规则可以保留。
5. 补搜任务应该按收益/成本分配，而不是只按最近距离。

如果需要，可以给 `Task` 增加字段：

```python
uncovered_value: float = 0.0
priority_value: float = 0.0
score: float = 0.0
```

## 第六部分：目标发现、盘旋确认与续搜

当前 `TARGET_FOUND` 后已有盘旋确认逻辑，请保留，但优化确认后的续搜。

要求：

1. 发现目标后：

   * 当前 UAV 切换为 `CONFIRMING`；
   * 当前搜索任务重新入队；
   * 只保留尚未覆盖的航点；
   * UAV 执行目标周边盘旋路径。
2. 确认完成后：

   * UAV 切回 `IDLE`；
   * 不要机械恢复原任务；
   * 让它重新参与收益驱动拍卖；
   * 如果没有高价值任务，则返航。
3. 确认路径仍然不能穿越障碍或禁飞区。
4. 目标确认不应导致其他 UAV 大量重复搜索同一区域。

## 第七部分：指标与可视化

请在 metrics 或 snapshots 中增加以下字段，用来判断策略是否真的变智能：

```json
{
  "coverage_goal_met": true,
  "priority_goal_met": true,
  "supplemental_task_count": 3,
  "ignored_fragment_count": 12,
  "final_uncovered_cells": 40,
  "final_priority_uncovered_cells": 0,
  "valuable_supplemental_candidate_count": 2,
  "post_95_extra_time_s": 35,
  "post_95_extra_distance_m": 420
}
```

至少需要在 metrics 中增加：

* `supplemental_task_count`
* `ignored_uncovered_cells`
* `final_uncovered_cells`
* `final_priority_uncovered_cells`
* `post_95_extra_time_s`
* `post_95_extra_distance_m`

重点观察：

* 95% 覆盖后是否还飞了很久；
* 后期是否还跨地图补小碎片；
* 重点区域是否优先达标；
* UAV 是否长时间 IDLE；
* 多 UAV 是否重复覆盖严重。

## 第八部分：测试要求

请新增或修改单元测试，至少覆盖：

1. 全局覆盖率达到 `mission_complete_coverage_threshold` 且重点区域达标后，普通碎片不再生成补搜任务。
2. 重点区域未达标时，即使全局覆盖率超过阈值，也必须继续补搜重点区域。
3. 小于 `min_supplemental_cells` 的普通碎片会被忽略。
4. score 低于 `min_supplemental_score` 的远距离低价值区域不会生成补搜任务。
5. 空闲 UAV 在无高价值任务时会触发返航。
6. 补搜候选会排除正在执行路径的未来传感器覆盖区。
7. 任务分配后航线会根据 UAV 当前位置重排入口。
8. 目标发现后 UAV 能盘旋确认，确认完成后重新参与任务分配或返航。

## 第九部分：场景回归

运行：

```powershell
python -m pytest -q
```

以及：

```powershell
python -m uav_search.experiments.run_batch --scenarios basic multi_basic dynamic_basic multi_target_no_fly urban_multi_target wide_random_targets --output-dir runs/batch_strategy_v2
```

验收标准：

1. 所有场景 `no_fly_violations == 0`。
2. 重点区域覆盖率 `>= 0.98`，除非场景中不存在重点区域。
3. 全局覆盖率通常 `>= 0.92`，不再强制要求 1.0。
4. `target_found_count` 和 `confirm_done_count` 不下降。
5. `post_95_extra_time_s` 相比旧策略明显降低。
6. 多目标场景中 `path_efficiency` 应比旧版本提高，尤其是：

   * `multi_target_no_fly`
   * `urban_multi_target`
   * `wide_random_targets`
7. UAV 不应长时间 IDLE 等待，除非已经返航或没有高价值任务。

## 开发约束

1. 不要引入强化学习。
2. 不要重写整个项目。
3. 不要破坏现有目标确认、地图更新、低电量返航、禁飞区避障逻辑。
4. 优先做可解释、可测试的规则策略。
5. 每一步改动后运行测试。
6. 如果需要新增函数，尽量放在现有模块中，不要过度抽象。
7. 最终请说明：

   * 改了哪些文件；
   * 策略相比旧版本解决了什么问题；
   * 哪些问题仍然没有解决；
   * 如何通过可视化判断效果是否变好。
