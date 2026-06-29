import { afterEach, describe, expect, it, vi } from "vitest";
import { createSimulationClient } from "./client";

describe("simulation API client", () => {
  afterEach(() => {
    vi.useRealTimers();
  });

  it("posts reset requests through the configured backend base URL with optional algorithm version", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true,
      json: async () => ({ run_id: "run_1", map: {} }),
    });
    const client = createSimulationClient({
      baseUrl: "http://backend.test",
      fetchImpl: fetchMock,
    });

    await client.resetSimulation("config/default.yaml", "config/scenarios/area_search_1uav.yaml", "adaptive_component_sweep_v1");

    expect(fetchMock).toHaveBeenCalledWith(
      "http://backend.test/api/sim/reset",
      expect.objectContaining({
        method: "POST",
        body: JSON.stringify({
          config_path: "config/default.yaml",
          scenario_path: "config/scenarios/area_search_1uav.yaml",
          algorithm_version: "adaptive_component_sweep_v1",
        }),
      }),
    );
  });

  it("posts mission draft reset requests to reset_custom", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true,
      json: async () => ({ run_id: "run_custom", map: {} }),
    });
    const client = createSimulationClient({
      baseUrl: "http://backend.test",
      fetchImpl: fetchMock,
    });
    const mission = {
      draftUavs: [
        {
          id: "uav_01",
          home_position: { x: 4, y: 5 },
          initial_position: { x: 4, y: 5 },
          sensor_radius_cells: 3,
          speed_mps: 11,
          battery: 0.8,
        },
      ],
      draftObstacles: [],
      draftSearchRegion: { x: 0, y: 0, width: 50, height: 50 },
      draftPriorityRegions: [],
      draftMapConfig: { width_cells: 50, height_cells: 50, resolution_m: 10 },
    };

    await client.resetCustomSimulation("config/default.yaml", "config/scenarios/area_search_1uav.yaml", mission, "adaptive_component_sweep_v1");

    expect(fetchMock).toHaveBeenCalledWith(
      "http://backend.test/api/sim/reset_custom",
      expect.objectContaining({
        method: "POST",
        body: JSON.stringify({
          config_path: "config/default.yaml",
          scenario_path: "config/scenarios/area_search_1uav.yaml",
          algorithm_version: "adaptive_component_sweep_v1",
          mission,
        }),
      }),
    );
  });

  it("fetches algorithm metadata", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true,
      json: async () => ({
        algorithms: [{ version: "adaptive_component_sweep_v1", label: "Adaptive", description: "Default" }],
        default_version: "adaptive_component_sweep_v1",
      }),
    });
    const client = createSimulationClient({
      baseUrl: "http://backend.test",
      fetchImpl: fetchMock,
    });

    const result = await client.getAlgorithms();

    expect(fetchMock).toHaveBeenCalledWith("http://backend.test/api/algorithms", expect.any(Object));
    expect(result.default_version).toBe("adaptive_component_sweep_v1");
  });

  it("opens websocket connections using the matching ws protocol", () => {
    const sockets: Array<{ url: string; close: () => void }> = [];
    class FakeWebSocket {
      onmessage: ((event: MessageEvent<string>) => void) | null = null;
      onopen: (() => void) | null = null;
      onclose: (() => void) | null = null;
      onerror: (() => void) | null = null;
      constructor(public url: string) {
        sockets.push({ url, close: () => this.close() });
      }
      close() {
        this.onclose?.();
      }
    }
    const client = createSimulationClient({
      baseUrl: "https://backend.test",
      WebSocketCtor: FakeWebSocket as unknown as typeof WebSocket,
    });

    const close = client.connectWebSocket(() => undefined);
    close();

    expect(sockets[0].url).toBe("wss://backend.test/ws/sim");
  });

  it("reconnects websocket connections and reports reconnecting status", async () => {
    vi.useFakeTimers();
    const statuses: string[] = [];
    const sockets: FakeSocket[] = [];
    class FakeSocket {
      onmessage: ((event: MessageEvent<string>) => void) | null = null;
      onopen: (() => void) | null = null;
      onclose: (() => void) | null = null;
      onerror: (() => void) | null = null;
      constructor(public url: string) {
        sockets.push(this);
      }
      close() {
        this.onclose?.();
      }
    }
    const client = createSimulationClient({
      baseUrl: "http://backend.test",
      WebSocketCtor: FakeSocket as unknown as typeof WebSocket,
    });

    const close = client.connectWebSocket(() => undefined, (status) => statuses.push(status));
    sockets[0].onopen?.();
    sockets[0].onclose?.();
    await vi.advanceTimersByTimeAsync(500);
    sockets[1].onopen?.();
    close();

    expect(statuses).toEqual(["connected", "reconnecting", "connected", "offline"]);
    expect(sockets).toHaveLength(2);
  });

  it("reports websocket JSON parse errors without crashing", () => {
    const errors: string[] = [];
    let socket: FakeSocket | undefined;
    class FakeSocket {
      onmessage: ((event: MessageEvent<string>) => void) | null = null;
      onopen: (() => void) | null = null;
      onclose: (() => void) | null = null;
      onerror: (() => void) | null = null;
      constructor(public url: string) {
        socket = this;
      }
      close() {
        this.onclose?.();
      }
    }
    const client = createSimulationClient({
      baseUrl: "http://backend.test",
      WebSocketCtor: FakeSocket as unknown as typeof WebSocket,
    });

    const close = client.connectWebSocket(() => undefined, undefined, (message) => errors.push(message));
    socket?.onmessage?.({ data: "not json" } as MessageEvent<string>);
    close();

    expect(errors[0]).toContain("Invalid WebSocket state payload");
  });

  it("throws readable errors for rejected responses", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: false,
      status: 404,
      json: async () => ({ detail: "scenario_path not found" }),
    });
    const client = createSimulationClient({
      baseUrl: "http://backend.test",
      fetchImpl: fetchMock,
    });

    await expect(client.getState(true, "full")).rejects.toThrow("scenario_path not found");
  });

  it("posts export requests and returns the exported file list", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true,
      json: async () => ({ run_id: "run_export", export_dir: "runs/web_exports/run_export", files: ["summary.json"] }),
    });
    const client = createSimulationClient({
      baseUrl: "http://backend.test",
      fetchImpl: fetchMock,
    });

    const result = await client.exportRun();

    expect(fetchMock).toHaveBeenCalledWith(
      "http://backend.test/api/sim/export",
      expect.objectContaining({ method: "POST" }),
    );
    expect(result.files).toEqual(["summary.json"]);
  });
});
