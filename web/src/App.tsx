import { Activity, AlertTriangle } from "lucide-react";
import { CommandLog } from "./components/CommandLog";
import { ControlPanel } from "./components/ControlPanel";
import { EventLog } from "./components/EventLog";
import { MapCanvas } from "./components/MapCanvas";
import { MetricsPanel } from "./components/MetricsPanel";
import { TaskTargetPanel } from "./components/TaskTargetPanel";
import { Toolbar } from "./components/Toolbar";
import { UavPanel } from "./components/UavPanel";
import { useSimulation } from "./hooks/useSimulation";

export function App() {
  const sim = useSimulation();
  const state = sim.currentState;

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
          <ControlPanel sim={sim} />
          <Toolbar sim={sim} />
        </aside>

        <section className="map-section">
          <MapCanvas sim={sim} />
        </section>

        <aside className="right-rail panel-stack">
          <MetricsPanel state={state} fullMetrics={sim.fullMetrics} onFetchMetrics={sim.fetchMetrics} />
          <UavPanel
            state={state}
            activeCommands={state?.active_commands || []}
            selectedUavId={sim.selectedUavId}
            onSelectUav={sim.setSelectedUavId}
            onSetOnline={sim.setUavOnlineState}
          />
          <TaskTargetPanel state={state} />
        </aside>
      </section>

      <section className="log-grid">
        <CommandLog
          entries={sim.commandLog}
          selectedCommandId={sim.selectedCommandId}
          onSelectCommand={sim.setSelectedCommandId}
          onClearLogs={sim.clearFrontEndLogs}
        />
        <EventLog state={state} eventLog={sim.eventLog} />
      </section>
    </main>
  );
}
