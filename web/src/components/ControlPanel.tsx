import { Pause, Play, RotateCcw, SkipForward, RefreshCw } from "lucide-react";
import { useState } from "react";
import type { UseSimulationResult } from "../hooks/useSimulation";

interface Props {
  sim: UseSimulationResult;
}

export function ControlPanel({ sim }: Props) {
  const [stepCount, setStepCount] = useState(10);
  const state = sim.currentState;

  return (
    <section className="panel">
      <h2>Control</h2>
      <label className="field">
        <span>Scenario</span>
        <select value={sim.selectedScenario || ""} onChange={(event) => sim.setSelectedScenario(event.target.value)}>
          {sim.scenarios.map((scenario) => (
            <option key={scenario.path} value={scenario.path}>
              {scenario.name}
            </option>
          ))}
        </select>
      </label>

      <div className="button-grid">
        <button onClick={sim.reset}>
          <RotateCcw size={15} /> Reset
        </button>
        <button onClick={() => sim.step(1)}>
          <SkipForward size={15} /> Step 1
        </button>
        <button onClick={() => sim.start(100)}>
          <Play size={15} /> Start
        </button>
        <button onClick={sim.pause}>
          <Pause size={15} /> Pause
        </button>
      </div>

      <div className="inline-control">
        <input
          type="number"
          min={1}
          max={100}
          value={stepCount}
          onChange={(event) => setStepCount(clamp(Number(event.target.value) || 1, 1, 100))}
        />
        <button onClick={() => sim.step(stepCount)}>
          <SkipForward size={15} /> Step N
        </button>
      </div>

      <button className="wide-button" onClick={sim.refreshFullState}>
        <RefreshCw size={15} /> Refresh full state
      </button>

      <dl className="stat-list">
        <div>
          <dt>time_s</dt>
          <dd>{fmt(state?.time_s)}</dd>
        </div>
        <div>
          <dt>tick</dt>
          <dd>{state?.tick ?? "-"}</dd>
        </div>
        <div>
          <dt>run_id</dt>
          <dd className="mono compact">{state?.run_id || "-"}</dd>
        </div>
        <div>
          <dt>running</dt>
          <dd>{state?.running ? "true" : "false"}</dd>
        </div>
      </dl>
    </section>
  );
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function fmt(value?: number): string {
  return typeof value === "number" ? value.toFixed(1) : "-";
}
