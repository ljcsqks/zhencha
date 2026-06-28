# Web MVP / Simulation Console

Phase 7a adds a single-user React + Vite + TypeScript console over the FastAPI Simulation Server. The Web app does not call scheduler, map, or fleet internals. It uses HTTP and WebSocket endpoints exposed by `uav_search.server.app`.

Phase 7c adds demo presets, server-side run export, local snapshots replay, and an acceptance panel for PASS/WARN/FAIL demo checks. See `docs/demo-runbook.md` for the operator script.

Phase 8b-6a adds a lightweight Algorithm selector for research and demo comparison. Phase 8b-6b promotes `adaptive_component_sweep_v1` as the default planner; selecting another algorithm in the Web console only affects the next `Reset` request and does not rewrite `config/default.yaml`.

## Start Backend

```bash
python -m uvicorn uav_search.server.app:app --reload
```

The backend defaults to `http://127.0.0.1:8000`.

## Start Frontend

```bash
cd web
npm install
npm run dev
```

The frontend defaults to `http://127.0.0.1:5173` and connects to `http://127.0.0.1:8000`.

To use a different backend:

```bash
set VITE_API_BASE_URL=http://127.0.0.1:8000
npm run dev
```

## Common Operations

- Select a scenario and press `Reset`.
- Select an Algorithm when comparing planners. `Adaptive Component Sweep v1` is the current default, `Baseline Sparse Boustrophedon` is the internal comparison baseline, and `Segment Sweep v1` is useful for checking scanline segment behavior. Changing the selector does not affect a running simulation until `Reset`.
- Use `Step 1`, `Step N`, `Start`, and `Pause` to control simulation time.
- Use `Refresh full state` when a full map resync is needed.
- Use `Fetch metrics` for the full `compute_metrics` result. Normal tick states use a lightweight metrics summary.
- Use `Export Run` after at least one step to write replay and metric artifacts under `runs/web_exports/<run_id>/`.
- Load an exported `snapshots.json` in the Replay panel to inspect a completed run without controlling the live simulation.
- Select `Inject Target`, then click the map to send a `TARGET_FOUND` event.
- Select `Add Obstacle` or `Remove Obstacle`, then drag a rectangle on the map to send a `MAP_UPDATE`.
- Use `Offline` and `Recover` in the UAV panel to send `UAV_OFFLINE` and `UAV_RECOVERED`.
- Watch command, ack, event, task, target, and metric panels for the resulting loop closure.
- Click a UAV row to highlight its history and active path. Click a command row to highlight that command path.
- Use `Clear logs` to clear frontend command/event history, and `Auto-follow latest UAV` to keep the newest UAV selected.

## State Behavior

- `reset` and the first WebSocket frame are full states and refresh the full map.
- Normal tick frames are lite states and do not include full map arrays.
- Lite tick frames include `coverage_changed_cells`; the frontend patches the retained full map's `coverage_count` and `search_confidence` so the heatmap updates in realtime without fetching the full map each frame.
- If a lite state contains terrain `changed_cells` from `MAP_UPDATE`, the frontend requests `GET /api/sim/state?include_map=true&state_level=full`.
- If a new `run_id` arrives, old trajectories and logs are cleared.
- Command and ack lifecycle is keyed by `command_id`. Late acks update earlier command rows, ack-only records remain visible, and the frontend keeps the most recent 300 command ids.
- Event lifecycle is keyed by `event_id`. Queued and handled records update the same row, with pending, recent, and history sections kept distinct in the event panel.
- Lite states include `active_commands` with `remaining_path` and progress. The map draws active remaining paths first, then falls back to recently issued command paths.
- While the simulation is running, event injection only queues the event and applies the returned state. While paused, the frontend queues the event and then performs one step so the result is visible immediately.
- WebSocket status is shown as `connected`, `reconnecting`, or `offline`. Reconnect uses capped backoff and refreshes full state after reconnection; invalid WebSocket JSON is reported as an error without crashing the UI.
- Replay mode uses local JSON only. While replay is active, real-time controls and event injection are hidden to avoid mixing replay inspection with live simulation.
- The current `algorithm_version` is included in state, metrics, and exported summaries. This is primarily for development comparison and demo traceability.

## Known Limits

- Single runtime and single-user operation.
- Canvas map rendering only; no GIS framework.
- Lite states omit full map arrays.
- MAP_UPDATE triggers a frontend full-state refresh after the next tick because terrain/passability changes are not sent as compact map patches yet.
- No Docker in this phase.
- Search algorithm selection is available for comparison, but this console does not tune or optimize algorithms at runtime.
- No target strike logic.
- No 3D view.
