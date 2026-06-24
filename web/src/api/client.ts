import type { EventRequest, EventResponse, ScenarioListResponse, SimulationState, StateLevel } from "../types/sim";

type FetchLike = typeof fetch;
export type WebSocketStatus = "connected" | "reconnecting" | "offline";

interface ClientOptions {
  baseUrl?: string;
  fetchImpl?: FetchLike;
  WebSocketCtor?: typeof WebSocket;
}

export interface SimulationClient {
  getHealth(): Promise<Record<string, unknown>>;
  getScenarios(): Promise<ScenarioListResponse>;
  resetSimulation(configPath: string, scenarioPath: string): Promise<SimulationState>;
  stepSimulation(steps: number): Promise<SimulationState>;
  startSimulation(tickIntervalMs: number): Promise<SimulationState>;
  pauseSimulation(): Promise<SimulationState>;
  getState(includeMap: boolean, stateLevel: StateLevel): Promise<SimulationState>;
  getMetrics(): Promise<Record<string, unknown>>;
  postEvent(event: EventRequest): Promise<EventResponse>;
  connectWebSocket(
    onState: (state: SimulationState) => void,
    onStatus?: (status: WebSocketStatus) => void,
    onError?: (message: string) => void,
  ): () => void;
}

const DEFAULT_BASE_URL = import.meta.env.VITE_API_BASE_URL || "http://127.0.0.1:8000";

export const apiBaseUrl = DEFAULT_BASE_URL;

export function createSimulationClient(options: ClientOptions = {}): SimulationClient {
  const baseUrl = trimTrailingSlash(options.baseUrl || DEFAULT_BASE_URL);
  const fetchImpl = options.fetchImpl || fetch;
  const WebSocketImpl = options.WebSocketCtor || WebSocket;

  async function request<T>(path: string, init?: RequestInit): Promise<T> {
    const response = await fetchImpl(`${baseUrl}${path}`, {
      headers: { "Content-Type": "application/json", ...(init?.headers || {}) },
      ...init,
    });
    if (!response.ok) {
      let detail = `${response.status} ${response.statusText}`;
      try {
        const payload = await response.json();
        detail = String(payload.detail || detail);
      } catch {
        // Keep the HTTP status text when the body is not JSON.
      }
      throw new Error(detail);
    }
    return (await response.json()) as T;
  }

  return {
    getHealth: () => request<Record<string, unknown>>("/api/health"),
    getScenarios: () => request<ScenarioListResponse>("/api/scenarios"),
    resetSimulation: (configPath, scenarioPath) =>
      request<SimulationState>("/api/sim/reset", {
        method: "POST",
        body: JSON.stringify({ config_path: configPath, scenario_path: scenarioPath }),
      }),
    stepSimulation: (steps) =>
      request<SimulationState>("/api/sim/step", {
        method: "POST",
        body: JSON.stringify({ steps }),
      }),
    startSimulation: (tickIntervalMs) =>
      request<SimulationState>("/api/sim/start", {
        method: "POST",
        body: JSON.stringify({ tick_interval_ms: tickIntervalMs }),
      }),
    pauseSimulation: () => request<SimulationState>("/api/sim/pause", { method: "POST" }),
    getState: (includeMap, stateLevel) =>
      request<SimulationState>(`/api/sim/state?include_map=${includeMap ? "true" : "false"}&state_level=${stateLevel}`),
    getMetrics: () => request<Record<string, unknown>>("/api/sim/metrics"),
    postEvent: (event) =>
      request<EventResponse>("/api/sim/event", {
        method: "POST",
        body: JSON.stringify(event),
      }),
    connectWebSocket: (onState, onStatus, onError) => {
      let socket: WebSocket | undefined;
      let closedByCaller = false;
      let reconnectTimer: ReturnType<typeof setTimeout> | undefined;
      let reconnectAttempt = 0;

      const connect = () => {
        socket = new WebSocketImpl(`${toWebSocketBase(baseUrl)}/ws/sim`);
        socket.onopen = () => {
          reconnectAttempt = 0;
          onStatus?.("connected");
        };
        socket.onclose = () => {
          if (closedByCaller) {
            return;
          }
          onStatus?.("reconnecting");
          const delayMs = Math.min(5000, 500 * 2 ** reconnectAttempt);
          reconnectAttempt += 1;
          reconnectTimer = setTimeout(connect, delayMs);
        };
        socket.onerror = () => {
          if (!closedByCaller) {
            onStatus?.("reconnecting");
          }
        };
        socket.onmessage = (event) => {
          try {
            onState(JSON.parse(event.data) as SimulationState);
          } catch (err) {
            onError?.(`Invalid WebSocket state payload: ${err instanceof Error ? err.message : String(err)}`);
          }
        };
      };

      connect();
      return () => {
        closedByCaller = true;
        if (reconnectTimer) {
          clearTimeout(reconnectTimer);
        }
        socket?.close();
        onStatus?.("offline");
      };
    },
  };
}

export const simulationClient = createSimulationClient();

function trimTrailingSlash(value: string): string {
  return value.replace(/\/+$/, "");
}

function toWebSocketBase(baseUrl: string): string {
  if (baseUrl.startsWith("https://")) {
    return baseUrl.replace("https://", "wss://");
  }
  if (baseUrl.startsWith("http://")) {
    return baseUrl.replace("http://", "ws://");
  }
  return `ws://${baseUrl}`;
}
