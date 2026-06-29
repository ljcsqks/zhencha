import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { simulationClient } from "../api/client";
import {
  addDraftUav,
  canEditMissionDraft,
  createMissionDraftFromState,
  moveDraftUav,
  removeDraftUav,
  updateDraftUav,
} from "../mission/missionDraft";
import type { AlgorithmInfo, EventRequest, ExportResponse, GridPosition, MissionDraft, ScenarioInfo, SimulationState } from "../types/sim";
import {
  emptySimulationClientState,
  mergeSimulationState,
  shouldRefreshFullStateAfterEvent,
  shouldStepAfterEvent,
  type CommandLogEntry,
  type SimulationClientState,
} from "./simulationState";

export type ToolMode = "inspect" | "target" | "addUav" | "addObstacle" | "removeObstacle";

export interface SimulationActions {
  loadScenarios(): Promise<void>;
  reset(): Promise<void>;
  step(steps?: number): Promise<void>;
  start(intervalMs?: number): Promise<void>;
  pause(): Promise<void>;
  refreshFullState(): Promise<void>;
  fetchMetrics(): Promise<void>;
  exportRun(): Promise<void>;
  injectTarget(x: number, y: number): Promise<void>;
  updateObstacle(operation: "add_obstacle" | "remove_obstacle", x: number, y: number, width: number, height: number): Promise<void>;
  setUavOnlineState(uavId: string, online: boolean): Promise<void>;
  resetDraftFromState(): void;
  addDraftUavAt(position: GridPosition): void;
  moveDraftUavTo(uavId: string, position: GridPosition): void;
  removeDraftUavById(uavId: string): void;
  updateDraftUavFields(uavId: string, patch: Partial<MissionDraft["draftUavs"][number]>): void;
}

export interface UseSimulationResult extends SimulationClientState, SimulationActions {
  scenarios: ScenarioInfo[];
  selectedScenario?: string;
  setSelectedScenario(path: string): void;
  algorithms: AlgorithmInfo[];
  selectedAlgorithmVersion?: string;
  setSelectedAlgorithmVersion(version: string): void;
  connected: boolean;
  connectionStatus: "connected" | "reconnecting" | "offline";
  running: boolean;
  error?: string;
  clearError(): void;
  commandLog: CommandLogEntry[];
  toolMode: ToolMode;
  setToolMode(mode: ToolMode): void;
  showCoverage: boolean;
  setShowCoverage(value: boolean): void;
  showPlannedPath: boolean;
  setShowPlannedPath(value: boolean): void;
  showHistoryPath: boolean;
  setShowHistoryPath(value: boolean): void;
  showGrid: boolean;
  setShowGrid(value: boolean): void;
  selectedUavId?: string;
  setSelectedUavId(value: string | undefined): void;
  selectedCommandId?: string;
  setSelectedCommandId(value: string | undefined): void;
  autoFollowLatestUav: boolean;
  setAutoFollowLatestUav(value: boolean): void;
  fullMetrics?: Record<string, unknown>;
  exportResult?: ExportResponse;
  missionDraft: MissionDraft;
  draftEditable: boolean;
  busy: boolean;
  clearFrontEndLogs(): void;
}

export function useSimulation(): UseSimulationResult {
  const [clientState, setClientState] = useState<SimulationClientState>(() => emptySimulationClientState());
  const [scenarios, setScenarios] = useState<ScenarioInfo[]>([]);
  const [selectedScenario, setSelectedScenario] = useState<string>();
  const [algorithms, setAlgorithms] = useState<AlgorithmInfo[]>([]);
  const [selectedAlgorithmVersion, setSelectedAlgorithmVersion] = useState<string>();
  const [connected, setConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState<"connected" | "reconnecting" | "offline">("offline");
  const [error, setError] = useState<string>();
  const [toolMode, setToolMode] = useState<ToolMode>("inspect");
  const [showCoverage, setShowCoverage] = useState(true);
  const [showPlannedPath, setShowPlannedPath] = useState(true);
  const [showHistoryPath, setShowHistoryPath] = useState(false);
  const [showGrid, setShowGrid] = useState(false);
  const [selectedUavId, setSelectedUavId] = useState<string | undefined>();
  const [selectedCommandId, setSelectedCommandId] = useState<string | undefined>();
  const [autoFollowLatestUav, setAutoFollowLatestUav] = useState(false);
  const [fullMetrics, setFullMetrics] = useState<Record<string, unknown> | undefined>();
  const [exportResult, setExportResult] = useState<ExportResponse | undefined>();
  const [missionDraft, setMissionDraft] = useState<MissionDraft>(() => createMissionDraftFromState());
  const [draftDirty, setDraftDirty] = useState(false);
  const [busy, setBusy] = useState(false);
  const draftDirtyRef = useRef(false);
  const refreshingRef = useRef(false);
  const currentStateRef = useRef<SimulationState | undefined>(undefined);
  const hasConnectedRef = useRef(false);
  const inFlightCountRef = useRef(0);

  const beginRequest = useCallback(() => {
    inFlightCountRef.current += 1;
    setBusy(true);
  }, []);

  const endRequest = useCallback(() => {
    inFlightCountRef.current = Math.max(0, inFlightCountRef.current - 1);
    if (inFlightCountRef.current === 0) {
      setBusy(false);
    }
  }, []);

  const applyState = useCallback((state: SimulationState) => {
    const previousRunId = currentStateRef.current?.run_id;
    currentStateRef.current = state;
    if (previousRunId && previousRunId !== state.run_id) {
      setFullMetrics(undefined);
      setExportResult(undefined);
    }
    setClientState((previous) => mergeSimulationState(previous, state));
    if (state.map && (!draftDirtyRef.current || (previousRunId && previousRunId !== state.run_id))) {
      setMissionDraft(createMissionDraftFromState(state));
      setDraftDirty(false);
      draftDirtyRef.current = false;
    }
    if (autoFollowLatestUav && state.uavs.length > 0) {
      setSelectedUavId(state.uavs[state.uavs.length - 1].id);
    }
  }, [autoFollowLatestUav]);

  const runRequest = useCallback(
    async (request: () => Promise<SimulationState | void>) => {
      try {
        beginRequest();
        setError(undefined);
        const state = await request();
        if (state) {
          applyState(state);
        }
      } catch (err) {
        setError(err instanceof Error ? err.message : String(err));
      } finally {
        endRequest();
      }
    },
    [applyState, beginRequest, endRequest],
  );

  const loadScenarios = useCallback(async () => {
    try {
      const response = await simulationClient.getScenarios();
      setScenarios(response.scenarios);
      setSelectedScenario((current) => current || response.scenarios[0]?.path);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }, []);

  const loadAlgorithms = useCallback(async () => {
    try {
      const response = await simulationClient.getAlgorithms();
      setAlgorithms(response.algorithms);
      setSelectedAlgorithmVersion((current) => current || response.default_version);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }, []);

  const refreshFullState = useCallback(async () => {
    await runRequest(async () => simulationClient.getState(true, "full"));
  }, [runRequest]);

  const fetchMetrics = useCallback(async () => {
    try {
      beginRequest();
      setError(undefined);
      setFullMetrics(await simulationClient.getMetrics());
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      endRequest();
    }
  }, [beginRequest, endRequest]);

  const exportRun = useCallback(async () => {
    try {
      beginRequest();
      setError(undefined);
      setExportResult(await simulationClient.exportRun());
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      endRequest();
    }
  }, [beginRequest, endRequest]);

  const postEventAndStep = useCallback(
    async (event: EventRequest) => {
      try {
        beginRequest();
        setError(undefined);
        const response = await simulationClient.postEvent(event);
        applyState(response.state);
        if (shouldStepAfterEvent(currentStateRef.current)) {
          const state = await simulationClient.stepSimulation(1);
          applyState(state);
          if (shouldRefreshFullStateAfterEvent(event, state)) {
            const full = await simulationClient.getState(true, "full");
            applyState(full);
          }
        }
      } catch (err) {
        setError(err instanceof Error ? err.message : String(err));
      } finally {
        endRequest();
      }
    },
    [applyState, beginRequest, endRequest],
  );

  useEffect(() => {
    loadScenarios();
    loadAlgorithms();
    const close = simulationClient.connectWebSocket(
      applyState,
      (status) => {
        setConnectionStatus(status);
        setConnected(status === "connected");
        if (status === "connected") {
          if (hasConnectedRef.current) {
            simulationClient
              .getState(true, "full")
              .then(applyState)
              .catch((err) => setError(err instanceof Error ? err.message : String(err)));
          }
          hasConnectedRef.current = true;
        }
      },
      setError,
    );
    return close;
  }, [applyState, loadAlgorithms, loadScenarios]);

  useEffect(() => {
    if (!clientState.needsFullMapRefresh || refreshingRef.current) {
      return;
    }
    refreshingRef.current = true;
    simulationClient
      .getState(true, "full")
      .then(applyState)
      .catch((err) => setError(err instanceof Error ? err.message : String(err)))
      .finally(() => {
        refreshingRef.current = false;
      });
  }, [applyState, clientState.needsFullMapRefresh, clientState.currentState?.tick]);

  const actions = useMemo<SimulationActions>(
    () => ({
      loadScenarios,
      reset: () =>
        runRequest(async () => {
          const scenarioPath = selectedScenario || scenarios[0]?.path || "config/scenarios/area_search_1uav.yaml";
          return simulationClient.resetCustomSimulation("config/default.yaml", scenarioPath, missionDraft, selectedAlgorithmVersion);
        }),
      step: (steps = 1) => runRequest(async () => simulationClient.stepSimulation(steps)),
      start: (intervalMs = 100) => runRequest(async () => simulationClient.startSimulation(intervalMs)),
      pause: () => runRequest(async () => simulationClient.pauseSimulation()),
      refreshFullState,
      fetchMetrics,
      exportRun,
      injectTarget: (x, y) =>
        postEventAndStep({
          type: "TARGET_FOUND",
          source_uav_id: null,
          data: {
            target_id: `web_target_${Date.now()}`,
            position: { x, y },
            confidence: 0.85,
            target_type: "unknown",
            orbit_radius_cells: 2,
            orbit_laps: 1,
            dwell_s: 5,
          },
        }),
      updateObstacle: (operation, x, y, width, height) =>
        postEventAndStep({
          type: "MAP_UPDATE",
          data: { operation, x, y, width, height },
        }),
      setUavOnlineState: (uavId, online) =>
        postEventAndStep({
          type: online ? "UAV_RECOVERED" : "UAV_OFFLINE",
          source_uav_id: uavId,
          data: {},
        }),
      resetDraftFromState: () => {
        setMissionDraft(createMissionDraftFromState(currentStateRef.current));
        setDraftDirty(false);
        draftDirtyRef.current = false;
      },
      addDraftUavAt: (position) => {
        if (!canEditMissionDraft(currentStateRef.current)) return;
        setMissionDraft((current) => addDraftUav(current, position));
        setDraftDirty(true);
        draftDirtyRef.current = true;
      },
      moveDraftUavTo: (uavId, position) => {
        if (!canEditMissionDraft(currentStateRef.current)) return;
        setMissionDraft((current) => moveDraftUav(current, uavId, position));
        setDraftDirty(true);
        draftDirtyRef.current = true;
      },
      removeDraftUavById: (uavId) => {
        if (!canEditMissionDraft(currentStateRef.current)) return;
        setMissionDraft((current) => removeDraftUav(current, uavId));
        setDraftDirty(true);
        draftDirtyRef.current = true;
      },
      updateDraftUavFields: (uavId, patch) => {
        if (!canEditMissionDraft(currentStateRef.current)) return;
        setMissionDraft((current) => updateDraftUav(current, uavId, patch));
        setDraftDirty(true);
        draftDirtyRef.current = true;
      },
    }),
    [
      exportRun,
      fetchMetrics,
      loadScenarios,
      postEventAndStep,
      refreshFullState,
      runRequest,
      scenarios,
      selectedAlgorithmVersion,
      selectedScenario,
      missionDraft,
    ],
  );

  return {
    ...clientState,
    ...actions,
    scenarios,
    selectedScenario,
    setSelectedScenario,
    algorithms,
    selectedAlgorithmVersion,
    setSelectedAlgorithmVersion,
    connected,
    connectionStatus,
    running: Boolean(clientState.currentState?.running),
    error,
    clearError: () => setError(undefined),
    toolMode,
    setToolMode,
    showCoverage,
    setShowCoverage,
    showPlannedPath,
    setShowPlannedPath,
    showHistoryPath,
    setShowHistoryPath,
    showGrid,
    setShowGrid,
    selectedUavId,
    setSelectedUavId,
    selectedCommandId,
    setSelectedCommandId,
    autoFollowLatestUav,
    setAutoFollowLatestUav,
    fullMetrics,
    exportResult,
    missionDraft,
    draftEditable: canEditMissionDraft(clientState.currentState),
    busy,
    clearFrontEndLogs: () =>
      setClientState((previous) => ({
        ...previous,
        commandLog: [],
        commandById: {},
        ackByCommandId: {},
        commandOrder: [],
        eventLog: [],
        eventById: {},
      eventOrder: [],
      })),
  };
}
