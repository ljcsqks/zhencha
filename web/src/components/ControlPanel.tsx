import { BarChart3, Download, Pause, Play, RotateCcw, SkipForward, RefreshCw } from "lucide-react";
import { useState } from "react";
import type { UseSimulationResult } from "../hooks/useSimulation";

interface Props {
  sim: UseSimulationResult;
  variant?: "operator" | "developer";
}

export function ControlPanel({ sim, variant = "developer" }: Props) {
  const [stepCount, setStepCount] = useState(10);
  const state = sim.currentState;
  const operator = variant === "operator";

  return (
    <section className="panel">
      <h2>{operator ? "Mission Setup" : "Control"}</h2>
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
      <label className="field">
        <span>Algorithm</span>
        <select
          value={sim.selectedAlgorithmVersion || ""}
          onChange={(event) => sim.setSelectedAlgorithmVersion(event.target.value)}
          title={sim.algorithms.find((item) => item.version === sim.selectedAlgorithmVersion)?.description}
        >
          {sim.algorithms.map((algorithm) => (
            <option key={algorithm.version} value={algorithm.version} title={algorithm.description}>
              {algorithm.label}
            </option>
          ))}
        </select>
        {!operator && (
          <small className="field-note">
            {sim.algorithms.find((item) => item.version === sim.selectedAlgorithmVersion)?.description ||
              "Research comparison only. Reset applies the selected algorithm."}
          </small>
        )}
      </label>
      <div className="algorithm-status">
        <span>Current algorithm</span>
        <strong className="mono compact">{state?.algorithm_version || "-"}</strong>
        {state?.running && sim.selectedAlgorithmVersion && sim.selectedAlgorithmVersion !== state.algorithm_version && (
          <small>Reset required to apply</small>
        )}
      </div>

      <div className="button-grid">
        <button onClick={sim.reset} disabled={sim.busy}>
          <RotateCcw size={15} /> Reset Custom
        </button>
        <button onClick={() => sim.step(1)} disabled={sim.busy}>
          <SkipForward size={15} /> Step 1
        </button>
        <button onClick={() => sim.start(100)} disabled={sim.busy}>
          <Play size={15} /> Start
        </button>
        <button onClick={sim.pause} disabled={sim.busy}>
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
        <button onClick={() => sim.step(stepCount)} disabled={sim.busy}>
          <SkipForward size={15} /> Step N
        </button>
      </div>

      {!operator && (
        <>
          <button className="wide-button" onClick={sim.refreshFullState} disabled={sim.busy}>
            <RefreshCw size={15} /> Refresh full state
          </button>
          <button className="wide-button" onClick={sim.fetchMetrics} disabled={sim.busy}>
            <BarChart3 size={15} /> Fetch metrics
          </button>
        </>
      )}
      <button className="wide-button" onClick={sim.exportRun} disabled={sim.busy}>
        <Download size={15} /> Export Run
      </button>

      {sim.exportResult && (
        <div className="export-result">
          <strong>Exported</strong>
          <span className="mono compact">{sim.exportResult.export_dir}</span>
          <small>{sim.exportResult.files.join(", ")}</small>
        </div>
      )}

      {!operator && (
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
      )}
    </section>
  );
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function fmt(value?: number): string {
  return typeof value === "number" ? value.toFixed(1) : "-";
}
