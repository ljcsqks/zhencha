import { Plus, RefreshCw, Trash2 } from "lucide-react";
import type { UseSimulationResult } from "../hooks/useSimulation";

interface Props {
  sim: UseSimulationResult;
}

export function MissionDraftPanel({ sim }: Props) {
  const draft = sim.missionDraft;
  const selectedDraft = draft.draftUavs.find((uav) => uav.id === sim.selectedUavId) || draft.draftUavs.at(-1);
  const map = draft.draftMapConfig;

  return (
    <section className="panel mission-draft-panel">
      <div className="panel-heading">
        <h2>Mission Draft</h2>
        <span className={`status-pill ${sim.draftEditable ? "idle" : "bad"}`}>{sim.draftEditable ? "editable" : "locked"}</span>
      </div>
      <div className="draft-summary">
        <span>UAVs</span>
        <strong>{draft.draftUavs.length}</strong>
        <span>Map</span>
        <strong>{map ? `${map.width_cells} x ${map.height_cells}` : "-"}</strong>
      </div>
      <div className="button-grid">
        <button onClick={() => sim.setToolMode("addUav")} disabled={!sim.draftEditable} className={sim.toolMode === "addUav" ? "selected" : ""}>
          <Plus size={15} /> Add UAV
        </button>
        <button onClick={sim.resetDraftFromState} disabled={!sim.draftEditable}>
          <RefreshCw size={15} /> Sync
        </button>
      </div>

      {selectedDraft ? (
        <div className="draft-editor">
          <div className="draft-editor-head">
            <strong>{selectedDraft.id}</strong>
            <button onClick={() => sim.removeDraftUavById(selectedDraft.id)} disabled={!sim.draftEditable || draft.draftUavs.length <= 1} title="Delete UAV">
              <Trash2 size={15} />
            </button>
          </div>
          <div className="field-grid two">
            <NumberField
              label="home x"
              value={selectedDraft.home_position.x}
              disabled={!sim.draftEditable}
              onChange={(value) =>
                sim.moveDraftUavTo(selectedDraft.id, { x: value, y: selectedDraft.initial_position.y })
              }
            />
            <NumberField
              label="home y"
              value={selectedDraft.home_position.y}
              disabled={!sim.draftEditable}
              onChange={(value) =>
                sim.moveDraftUavTo(selectedDraft.id, { x: selectedDraft.initial_position.x, y: value })
              }
            />
            <NumberField
              label="sensor"
              value={selectedDraft.sensor_radius_cells}
              min={1}
              disabled={!sim.draftEditable}
              onChange={(value) => sim.updateDraftUavFields(selectedDraft.id, { sensor_radius_cells: value })}
            />
            <NumberField
              label="speed"
              value={selectedDraft.speed_mps}
              min={1}
              step={0.5}
              disabled={!sim.draftEditable}
              onChange={(value) => sim.updateDraftUavFields(selectedDraft.id, { speed_mps: value })}
            />
            <NumberField
              label="battery"
              value={selectedDraft.battery}
              min={0}
              max={1}
              step={0.05}
              disabled={!sim.draftEditable}
              onChange={(value) => sim.updateDraftUavFields(selectedDraft.id, { battery: value })}
            />
          </div>
        </div>
      ) : (
        <span className="empty">No draft UAVs yet.</span>
      )}

      <div className="draft-uav-list">
        {draft.draftUavs.map((uav) => (
          <button
            key={uav.id}
            className={sim.selectedUavId === uav.id ? "draft-uav-pill selected" : "draft-uav-pill"}
            onClick={() => sim.setSelectedUavId(sim.selectedUavId === uav.id ? undefined : uav.id)}
          >
            <span>{uav.id}</span>
            <small>
              ({uav.initial_position.x}, {uav.initial_position.y})
            </small>
          </button>
        ))}
      </div>
    </section>
  );
}

function NumberField({
  label,
  value,
  min,
  max,
  step = 1,
  disabled,
  onChange,
}: {
  label: string;
  value: number;
  min?: number;
  max?: number;
  step?: number;
  disabled: boolean;
  onChange(value: number): void;
}) {
  return (
    <label className="field compact-field">
      <span>{label}</span>
      <input
        type="number"
        value={value}
        min={min}
        max={max}
        step={step}
        disabled={disabled}
        onChange={(event) => {
          const next = Number(event.target.value);
          if (Number.isFinite(next)) {
            onChange(clamp(next, min, max));
          }
        }}
      />
    </label>
  );
}

function clamp(value: number, min?: number, max?: number): number {
  const lower = min ?? Number.NEGATIVE_INFINITY;
  const upper = max ?? Number.POSITIVE_INFINITY;
  return Math.min(upper, Math.max(lower, value));
}
