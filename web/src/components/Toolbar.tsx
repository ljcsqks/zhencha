import { Building2, Crosshair, Eye, Eraser, Grid3X3, History, Map, Plane, Route, ShieldPlus } from "lucide-react";
import type { ToolMode, UseSimulationResult } from "../hooks/useSimulation";

interface Props {
  sim: UseSimulationResult;
}

const modes: Array<{ id: ToolMode; label: string; icon: typeof Eye }> = [
  { id: "inspect", label: "Inspect", icon: Eye },
  { id: "target", label: "Inject Target", icon: Crosshair },
  { id: "addUav", label: "Add UAV", icon: Plane },
  { id: "addObstacle", label: "Add Obstacle", icon: ShieldPlus },
  { id: "removeObstacle", label: "Remove Obstacle", icon: Eraser },
  { id: "modelBuilding", label: "Model Building", icon: Building2 },
];

export function Toolbar({ sim }: Props) {
  return (
    <section className="panel">
      <h2>Tools</h2>
      <div className="tool-list">
        {modes.map((mode) => {
          const Icon = mode.icon;
          return (
            <button
              key={mode.id}
              className={sim.toolMode === mode.id ? "selected" : ""}
              onClick={() => sim.setToolMode(mode.id)}
              title={mode.label}
            >
              <Icon size={16} />
              {mode.label}
            </button>
          );
        })}
      </div>

      {sim.toolMode === "modelBuilding" && (
        <div className="tool-options">
          <label>
            UAVs
            <input
              type="number"
              min={1}
              max={4}
              value={sim.modelingUavCount}
              onChange={(event) => sim.setModelingUavCount(Number(event.target.value))}
            />
          </label>
          <label>
            Standoff
            <input
              type="number"
              min={1}
              max={8}
              value={sim.modelingStandoffCells}
              onChange={(event) => sim.setModelingStandoffCells(Number(event.target.value))}
            />
          </label>
          <label>
            Laps
            <input
              type="number"
              min={1}
              max={4}
              value={sim.modelingLaps}
              onChange={(event) => sim.setModelingLaps(Number(event.target.value))}
            />
          </label>
          <label className="tool-option-inline">
            <input
              type="checkbox"
              checked={sim.modelingResumeSearch}
              onChange={(event) => sim.setModelingResumeSearch(event.target.checked)}
            />
            Resume search
          </label>
        </div>
      )}

      <div className="toggle-list">
        <label>
          <input type="checkbox" checked={sim.showCoverage} onChange={(event) => sim.setShowCoverage(event.target.checked)} />
          <Map size={15} /> Coverage
        </label>
        <label>
          <input type="checkbox" checked={sim.showGrid} onChange={(event) => sim.setShowGrid(event.target.checked)} />
          <Grid3X3 size={15} /> Grid
        </label>
        <label>
          <input
            type="checkbox"
            checked={sim.showPlannedPath}
            onChange={(event) => sim.setShowPlannedPath(event.target.checked)}
          />
          <Route size={15} /> Planned path
        </label>
        <label>
          <input
            type="checkbox"
            checked={sim.showHistoryPath}
            onChange={(event) => sim.setShowHistoryPath(event.target.checked)}
          />
          <History size={15} /> History path
        </label>
        <label>
          <input
            type="checkbox"
            checked={sim.autoFollowLatestUav}
            onChange={(event) => sim.setAutoFollowLatestUav(event.target.checked)}
          />
          <Crosshair size={15} /> Auto-follow latest UAV
        </label>
      </div>
    </section>
  );
}
