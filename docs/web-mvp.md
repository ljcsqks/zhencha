# Web MVP / Simulation Console

Phase 7a adds a single-user React + Vite + TypeScript console over the FastAPI Simulation Server. The Web app does not call scheduler, map, or fleet internals. It uses HTTP and WebSocket endpoints exposed by `uav_search.server.app`.

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
- Use `Step 1`, `Step N`, `Start`, and `Pause` to control simulation time.
- Use `Refresh full state` when a full map resync is needed.
- Use `Fetch metrics` for the full `compute_metrics` result. Normal tick states use a lightweight metrics summary.
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

## Known Limits

- Single runtime and single-user operation.
- Canvas map rendering only; no GIS framework.
- Lite states omit full map arrays.
- MAP_UPDATE triggers a frontend full-state refresh after the next tick because terrain/passability changes are not sent as compact map patches yet.
- No Docker in this phase.
- No search algorithm optimization in this phase.
- No target strike logic.
- No 3D view.
