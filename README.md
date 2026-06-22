# 多无人机区域协同搜索仿真

当前项目用于验证多架无人机在栅格地图上的区域搜索、重点区域优先覆盖、障碍物/禁飞区绕行、动态地图更新、补搜和返航逻辑。

## 环境

```powershell
pip install -r requirements.txt
```

本机可用命令：

```powershell
E:\anaconda\Scripts\pytest.exe -q
```

## 当前场景

所有预设场景均为 `500m x 500m`，分辨率 `10m/格`，即 `50 x 50` 栅格。场景暂不注入目标发现事件，先专注验证区域搜索策略。

| 场景 | UAV 数量 | 内容 |
|---|---:|---|
| `area_search_1uav` | 1 | 单机基础区域搜索，含重点区、静态障碍和动态障碍更新 |
| `area_search_2uav` | 2 | 双机上下分区搜索，含两个重点区、禁飞区和动态障碍 |
| `area_search_3uav` | 3 | 三机区域搜索，含中部障碍、禁飞区、重点区和动态障碍 |
| `area_search_4uav` | 4 | 四机复杂区域搜索，含多障碍、多禁飞区和重点区 |
| `area_search_5uav` | 5 | 五机高密度区域搜索，含多重点区、多禁飞区和动态障碍 |

## 单场景运行

```powershell
E:\anaconda\python.exe -m uav_search.main --config config/default.yaml --scenario config/scenarios/area_search_1uav.yaml --output runs/area_search_1uav_snapshots.json --image runs/area_search_1uav_view.png --metrics runs/area_search_1uav_metrics.json --report-dir runs/area_search_1uav_report
```

实时播放：

```powershell
E:\anaconda\python.exe -m uav_search.main --config config/default.yaml --scenario config/scenarios/area_search_3uav.yaml --output runs/area_search_3uav_snapshots.json --play --play-interval-ms 100
```

## 批量运行全部 500x500 场景

```powershell
E:\anaconda\python.exe -m uav_search.experiments.run_batch --scenarios area_search_1uav area_search_2uav area_search_3uav area_search_4uav area_search_5uav --output-dir runs/batch_area_search_500
```

每个场景会输出：

- `snapshots.json`
- `metrics.json`
- `final_view.png`
- `report/coverage_curve.png`
- `report/uav_trajectories.png`
- `report/event_timeline.png`

批量目录还会生成 `summary.json` 和 `summary.csv`。



E:\anaconda\python.exe -m uav_search.main --config config/default.yaml --scenario config/scenarios/area_search_1uav.yaml --output runs/area_search_1uav_snapshots.json --play --play-interval-ms 100
E:\anaconda\python.exe -m uav_search.main --config config/default.yaml --scenario config/scenarios/area_search_2uav.yaml --output runs/area_search_2uav_snapshots.json --play --play-interval-ms 100
E:\anaconda\python.exe -m uav_search.main --config config/default.yaml --scenario config/scenarios/area_search_3uav.yaml --output runs/area_search_3uav_snapshots.json --play --play-interval-ms 100
E:\anaconda\python.exe -m uav_search.main --config config/default.yaml --scenario config/scenarios/area_search_4uav.yaml --output runs/area_search_4uav_snapshots.json --play --play-interval-ms 100
E:\anaconda\python.exe -m uav_search.main --config config/default.yaml --scenario config/scenarios/area_search_5uav.yaml --output runs/area_search_5uav_snapshots.json --play --play-interval-ms 100
如果想播放慢一点，把 --play-interval-ms 100 改成 150 或 200。
