import { Clapperboard } from "lucide-react";
import type { UseSimulationResult } from "../hooks/useSimulation";

interface Props {
  sim: UseSimulationResult;
}

const DEMOS: Record<string, { title: string; expected: string; metrics: string }> = {
  demo_search_3uav: {
    title: "多机搜索 Multi-UAV Search",
    expected: "三架无人机分区搜索，覆盖重点区域并避开禁飞区。",
    metrics: "关注覆盖率、负载均衡和禁飞违规。",
  },
  demo_target_confirm: {
    title: "目标确认 Target Confirm",
    expected: "目标事件触发确认任务，确认后恢复搜索。",
    metrics: "关注确认成功率和任务恢复。",
  },
  demo_dynamic_obstacle: {
    title: "动态障碍 Dynamic Obstacle",
    expected: "地图更新后新增障碍，路径绕开变化区域。",
    metrics: "关注重规划次数、变化格和禁飞违规。",
  },
  demo_uav_offline_recover: {
    title: "无人机离线/恢复 UAV Offline / Recover",
    expected: "无人机离线后任务回收，恢复后重新参与任务。",
    metrics: "关注 UAV 状态、失败命令和任务恢复。",
  },
};

export function DemoPanel({ sim }: Props) {
  const demos = sim.scenarios.filter((scenario) => scenario.name.startsWith("demo_"));
  return (
    <section className="panel">
      <div className="panel-heading">
        <h2>预置演示</h2>
        <Clapperboard size={16} />
      </div>
      <p className="panel-note">算法可在 Control 面板选择，Reset 后生效。</p>
      <div className="demo-list">
        {demos.map((scenario) => {
          const info = DEMOS[scenario.name] || {
            title: scenario.name,
            expected: scenario.description || "预置演示场景。",
            metrics: "关注任务指标和日志。",
          };
          return (
            <button
              key={scenario.path}
              className={sim.selectedScenario === scenario.path ? "demo-card selected" : "demo-card"}
              onClick={() => sim.setSelectedScenario(scenario.path)}
            >
              <strong>{info.title}</strong>
              <span>{scenario.name}</span>
              <small>{info.expected}</small>
              <small>{info.metrics}</small>
            </button>
          );
        })}
        {demos.length === 0 && <span className="empty">未找到预置演示。</span>}
      </div>
    </section>
  );
}
