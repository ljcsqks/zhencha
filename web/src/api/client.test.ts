import { afterEach, describe, expect, it, vi } from "vitest";
import { createSimulationClient } from "./client";

describe("simulation API client", () => {
  afterEach(() => {
    vi.useRealTimers();
  });

  it("posts reset requests through the configured backend base URL", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true,
      json: async () => ({ run_id: "run_1", map: {} }),
    });
    const client = createSimulationClient({
      baseUrl: "http://backend.test",
      fetchImpl: fetchMock,
    });

    await client.resetSimulation("config/default.yaml", "config/scenarios/area_search_1uav.yaml");

    expect(fetchMock).toHaveBeenCalledWith(
      "http://backend.test/api/sim/reset",
      expect.objectContaining({
        method: "POST",
        body: JSON.stringify({
          config_path: "config/default.yaml",
          scenario_path: "config/scenarios/area_search_1uav.yaml",
        }),
      }),
    );
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
