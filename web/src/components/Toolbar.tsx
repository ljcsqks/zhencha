import { Building2, Crosshair, Eye, Eraser, Grid3X3, History, Map, Plane, Route, ShieldPlus } from "lucide-react";
import type { ModelingPostBehavior, ToolMode, UseSimulationResult } from "../hooks/useSimulation";

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
            与建筑保持距离
            <input
              type="number"
              min={1}
              max={8}
              value={sim.modelingStandoffCells}
              onChange={(event) => sim.setModelingStandoffCells(Number(event.target.value))}
            />
            <span>建模航线距离楼栋外轮廓的栅格数，例如 3 表示外扩 3 格飞行。</span>
          </label>
          <label>
            绕飞圈数
            <input
              type="number"
              min={1}
              max={4}
              value={sim.modelingLaps}
              onChange={(event) => sim.setModelingLaps(Number(event.target.value))}
            />
            <span>沿建模轨道重复飞行的次数；当前 2D 仿真中表示重复扫描，不代表真实高度层。</span>
          </label>
          <label>
            建模完成后
            <select
              value={sim.modelingPostBehavior}
              onChange={(event) => sim.setModelingPostBehavior(event.target.value as ModelingPostBehavior)}
            >
              <option value="return_home_when_no_resume">自动：恢复搜索或返航</option>
              <option value="hold">原地等待</option>
              <option value="return_home">返航</option>
            </select>
          </label>
          <label className="tool-option-inline">
            <input
              type="checkbox"
              checked={sim.modelingResumeSearch}
              onChange={(event) => sim.setModelingResumeSearch(event.target.checked)}
            />
            <span>
              建模后恢复搜索
              <small>如果建模任务打断了搜索，完成后恢复原搜索；如果搜索已完成，则默认返航。</small>
            </span>
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
