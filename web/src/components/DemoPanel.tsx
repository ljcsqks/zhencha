import { Clapperboard } from "lucide-react";
import type { UseSimulationResult } from "../hooks/useSimulation";

interface Props {
  sim: UseSimulationResult;
}

const DEMOS: Record<string, { title: string; expected: string; metrics: string }> = {
  demo_search_3uav: {
    title: "Multi-UAV Search",
    expected: "Three UAVs split the area, cover priority cells, and avoid restricted cells.",
    metrics: "Watch global coverage, workload balance, and no-fly violations.",
  },
  demo_target_confirm: {
    title: "Target Confirm",
    expected: "A source-free target event is assigned to a UAV, confirmed, then search resumes.",
    metrics: "Watch CONFIRM_TARGET acks, confirm success, and resume rate.",
  },
  demo_dynamic_obstacle: {
    title: "Dynamic Obstacle",
    expected: "A MAP_UPDATE adds an obstacle and paths are replanned around changed cells.",
    metrics: "Watch changed cells, replan count, and no-fly violations.",
  },
  demo_uav_offline_recover: {
    title: "UAV Offline / Recover",
    expected: "One UAV goes offline, later recovers, and commands avoid offline assignment.",
    metrics: "Watch UAV status, failed/cancelled acks, and task recovery.",
  },
};

export function DemoPanel({ sim }: Props) {
  const demos = sim.scenarios.filter((scenario) => scenario.name.startsWith("demo_"));
  return (
    <section className="panel">
      <div className="panel-heading">
        <h2>Demos</h2>
        <Clapperboard size={16} />
      </div>
      <p className="panel-note">算法可在 Control 面板中选择，Reset 后生效。</p>
      <div className="demo-list">
        {demos.map((scenario) => {
          const info = DEMOS[scenario.name] || {
            title: scenario.name,
            expected: scenario.description || "Demo scenario.",
            metrics: "Watch mission metrics and logs.",
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
        {demos.length === 0 && <span className="empty">No demo scenarios found.</span>}
      </div>
    </section>
  );
}
