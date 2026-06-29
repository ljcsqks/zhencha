import { ShieldCheck } from "lucide-react";
import { evaluateAcceptance } from "../acceptance/acceptance";
import type { CommandLogEntry } from "../hooks/simulationState";
import type { SimulationState } from "../types/sim";

interface Props {
  state?: SimulationState;
  commandLog: CommandLogEntry[];
}

export function AcceptancePanel({ state, commandLog }: Props) {
  const started = Boolean(state && (state.tick > 0 || state.running || state.global_coverage > 0 || commandLog.length > 0));
  const checks = evaluateAcceptance(state, commandLog);
  return (
    <section className="panel">
      <div className="panel-heading">
        <h2>Acceptance</h2>
        <ShieldCheck size={16} />
      </div>
      <div className="algorithm-status compact-status">
        <span>Algorithm</span>
        <strong className="mono compact">{state?.algorithm_version || "-"}</strong>
      </div>
      {!started && (
        <div className="acceptance-row neutral">
          <span>Not started</span>
          <strong>WAIT</strong>
          <small>Waiting for mission start</small>
        </div>
      )}
      <div className="acceptance-list">
        {started && checks.map((check) => (
          <div key={check.id} className={`acceptance-row ${check.status.toLowerCase()}`}>
            <span>{check.label}</span>
            <strong>{check.status}</strong>
            <small>{check.detail}</small>
          </div>
        ))}
      </div>
    </section>
  );
}
