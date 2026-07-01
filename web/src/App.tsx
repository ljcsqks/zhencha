import { Activity, AlertTriangle, Download, Pause, Play, RotateCcw, SkipForward } from "lucide-react";
import { useCallback, useMemo, useState } from "react";
import { AcceptancePanel } from "./components/AcceptancePanel";
import { CommandLog } from "./components/CommandLog";
import { ControlPanel } from "./components/ControlPanel";
import { DemoPanel } from "./components/DemoPanel";
import { EventLog } from "./components/EventLog";
import { MapCanvas } from "./components/MapCanvas";
import { MetricsPanel } from "./components/MetricsPanel";
import { MissionStatusPanel } from "./components/MissionStatusPanel";
import { MissionDraftPanel } from "./components/MissionDraftPanel";
import { ReplayPanel } from "./components/ReplayPanel";
import { TaskTargetPanel } from "./components/TaskTargetPanel";
import { Toolbar } from "./components/Toolbar";
import { UavPanel } from "./components/UavPanel";
import { useSimulation, type UseSimulationResult } from "./hooks/useSimulation";
import { mergeSimulationState, type SimulationClientState } from "./hooks/simulationState";
import type { SimulationState } from "./types/sim";

export function App() {
  const sim = useSimulation();
  const [uiMode, setUiMode] = useState<"operator" | "developer">("operator");
  const [diagnosticsOpen, setDiagnosticsOpen] = useState(false);
  const [replayClientState, setReplayClientState] = useState<SimulationClientState | undefined>();
  const replayActive = Boolean(replayClientState);
  const updateReplayState = useCallback((state: SimulationState | undefined) => {
    if (!state) {
      setReplayClientState(undefined);
      return;
    }
    setReplayClientState((previous) => mergeSimulationState(previous, state));
  }, []);
  const displaySim = useMemo<UseSimulationResult>(() => {
    if (!replayClientState) {
      return sim;
    }
    return {
      ...sim,
      ...replayClientState,
      running: false,
      toolMode: "inspect",
      injectTarget: async () => undefined,
      requestBuildingModel: async () => undefined,
      updateObstacle: async () => undefined,
      setUavOnlineState: async () => undefined,
      start: async () => undefined,
      step: async () => undefined,
    };
  }, [replayClientState, sim]);
  const state = displaySim.currentState;
  const operatorMode = uiMode === "operator";

  return (
    <main className={`app-shell ${operatorMode ? "operator-shell" : "developer-shell"}`}>
      <header className="topbar">
        <div className="brand-block">
          <Activity size={24} />
          <div>
            <h1>UAV Simulation Console</h1>
            <span>{state?.scenario_name || "waiting for backend"} / {state?.algorithm_version || sim.selectedAlgorithmVersion || "-"}</span>
          </div>
        </div>
        <div className="topbar-controls">
          <div className="mode-switch" aria-label="UI mode">
            <button aria-pressed={uiMode === "operator"} onClick={() => setUiMode("operator")}>Operator</button>
            <button aria-pressed={uiMode === "developer"} onClick={() => setUiMode("developer")}>Developer</button>
          </div>
          {operatorMode && !replayActive && (
            <div className="quick-actions" aria-label="Simulation shortcuts">
              <button onClick={sim.reset} disabled={sim.busy} title="Reset custom mission"><RotateCcw size={15} /> Reset</button>
              <button onClick={() => sim.step(1)} disabled={sim.busy} title="Step once"><SkipForward size={15} /> Step</button>
              <button onClick={() => sim.start(100)} disabled={sim.busy} title="Start simulation"><Play size={15} /> Start</button>
              <button onClick={sim.pause} disabled={sim.busy} title="Pause simulation"><Pause size={15} /> Pause</button>
              <button onClick={sim.exportRun} disabled={sim.busy} title="Export run"><Download size={15} /> Export</button>
            </div>
          )}
          <div className="status-row">
            <span className={`status-pill ${sim.connectionStatus === "connected" ? "ok" : sim.connectionStatus === "reconnecting" ? "idle" : "bad"}`}>
              WS {sim.connectionStatus}
            </span>
            <span className={`status-pill ${sim.running ? "ok" : "idle"}`}>{sim.running ? "running" : "paused"}</span>
          {replayActive && <span className="status-pill idle">Replay</span>}
            {!operatorMode && <span className="mono">{state?.run_id || "no run"}</span>}
          </div>
        </div>
      </header>

      {sim.error && (
        <div className="error-strip">
          <AlertTriangle size={16} />
          <span>{sim.error}</span>
          <button onClick={sim.clearError}>Dismiss</button>
        </div>
      )}

      <section className={operatorMode ? "operator-grid" : "console-grid"}>
        <aside className="left-rail panel-stack">
          {!operatorMode && <ReplayPanel active={replayActive} onReplayState={updateReplayState} onExit={() => setReplayClientState(undefined)} />}
          {!replayActive && (
            <>
              {!operatorMode && <DemoPanel sim={sim} />}
              <ControlPanel sim={sim} variant={operatorMode ? "operator" : "developer"} />
              <MissionDraftPanel sim={sim} />
              <Toolbar sim={sim} />
            </>
          )}
        </aside>

        <section className="map-section">
          <MapCanvas sim={displaySim} />
        </section>

        <aside className="right-rail panel-stack">
          {operatorMode ? (
            <MissionStatusPanel state={state} commandLog={displaySim.commandLog} />
          ) : (
            <>
              <AcceptancePanel state={state} commandLog={displaySim.commandLog} />
              <MetricsPanel state={state} fullMetrics={replayActive ? undefined : sim.fullMetrics} onFetchMetrics={sim.fetchMetrics} />
              <UavPanel
                state={state}
                activeCommands={state?.active_commands || []}
                selectedUavId={displaySim.selectedUavId}
                busy={displaySim.busy}
                onSelectUav={displaySim.setSelectedUavId}
                onSetOnline={displaySim.setUavOnlineState}
              />
              <TaskTargetPanel state={state} />
            </>
          )}
        </aside>
      </section>

      {operatorMode ? (
        <section className={`diagnostics-drawer ${diagnosticsOpen ? "open" : ""}`}>
          <button className="drawer-toggle" onClick={() => setDiagnosticsOpen((open) => !open)}>
            Logs / Diagnostics
          </button>
          {diagnosticsOpen && (
            <div className="drawer-content">
              <AcceptancePanel state={state} commandLog={displaySim.commandLog} />
              <CommandLog
                entries={displaySim.commandLog}
                selectedCommandId={displaySim.selectedCommandId}
                onSelectCommand={displaySim.setSelectedCommandId}
                onClearLogs={displaySim.clearFrontEndLogs}
              />
              <EventLog state={state} eventLog={displaySim.eventLog} />
            </div>
          )}
        </section>
      ) : (
        <section className="log-grid">
          <CommandLog
            entries={displaySim.commandLog}
            selectedCommandId={displaySim.selectedCommandId}
            onSelectCommand={displaySim.setSelectedCommandId}
            onClearLogs={displaySim.clearFrontEndLogs}
          />
          <EventLog state={state} eventLog={displaySim.eventLog} />
        </section>
      )}
    </main>
  );
}
