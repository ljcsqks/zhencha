import { Activity, AlertTriangle } from "lucide-react";
import { useCallback, useMemo, useState } from "react";
import { AcceptancePanel } from "./components/AcceptancePanel";
import { CommandLog } from "./components/CommandLog";
import { ControlPanel } from "./components/ControlPanel";
import { DemoPanel } from "./components/DemoPanel";
import { EventLog } from "./components/EventLog";
import { MapCanvas } from "./components/MapCanvas";
import { MetricsPanel } from "./components/MetricsPanel";
import { ReplayPanel } from "./components/ReplayPanel";
import { TaskTargetPanel } from "./components/TaskTargetPanel";
import { Toolbar } from "./components/Toolbar";
import { UavPanel } from "./components/UavPanel";
import { useSimulation, type UseSimulationResult } from "./hooks/useSimulation";
import { mergeSimulationState, type SimulationClientState } from "./hooks/simulationState";
import type { SimulationState } from "./types/sim";

export function App() {
  const sim = useSimulation();
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
      updateObstacle: async () => undefined,
      setUavOnlineState: async () => undefined,
      start: async () => undefined,
      step: async () => undefined,
    };
  }, [replayClientState, sim]);
  const state = displaySim.currentState;

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand-block">
          <Activity size={24} />
          <div>
            <h1>UAV Simulation Console</h1>
            <span>{state?.scenario_name || "waiting for backend"}</span>
          </div>
        </div>
        <div className="status-row">
          <span className={`status-pill ${sim.connectionStatus === "connected" ? "ok" : sim.connectionStatus === "reconnecting" ? "idle" : "bad"}`}>
            WS {sim.connectionStatus}
          </span>
        <span className={`status-pill ${sim.running ? "ok" : "idle"}`}>{sim.running ? "running" : "paused"}</span>
          {replayActive && <span className="status-pill idle">Replay</span>}
          <span className="mono">{state?.run_id || "no run"}</span>
        </div>
      </header>

      {sim.error && (
        <div className="error-strip">
          <AlertTriangle size={16} />
          <span>{sim.error}</span>
          <button onClick={sim.clearError}>Dismiss</button>
        </div>
      )}

      <section className="console-grid">
        <aside className="left-rail panel-stack">
          <ReplayPanel active={replayActive} onReplayState={updateReplayState} onExit={() => setReplayClientState(undefined)} />
          {!replayActive && (
            <>
              <DemoPanel sim={sim} />
              <ControlPanel sim={sim} />
              <Toolbar sim={sim} />
            </>
          )}
        </aside>

        <section className="map-section">
          <MapCanvas sim={displaySim} />
        </section>

        <aside className="right-rail panel-stack">
          <AcceptancePanel state={state} commandLog={displaySim.commandLog} />
          <MetricsPanel state={state} fullMetrics={replayActive ? undefined : sim.fullMetrics} onFetchMetrics={sim.fetchMetrics} />
          <UavPanel
            state={state}
            activeCommands={state?.active_commands || []}
            selectedUavId={displaySim.selectedUavId}
            onSelectUav={displaySim.setSelectedUavId}
            onSetOnline={displaySim.setUavOnlineState}
          />
          <TaskTargetPanel state={state} />
        </aside>
      </section>

      <section className="log-grid">
        <CommandLog
          entries={displaySim.commandLog}
          selectedCommandId={displaySim.selectedCommandId}
          onSelectCommand={displaySim.setSelectedCommandId}
          onClearLogs={displaySim.clearFrontEndLogs}
        />
        <EventLog state={state} eventLog={displaySim.eventLog} />
      </section>
    </main>
  );
}
