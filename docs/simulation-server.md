# Simulation Server

Phase 6 adds an in-process FastAPI service over the existing simulation runtime. The service does not call scheduler, map, or fleet internals directly; HTTP and WebSocket handlers operate through `SimulationRuntime`.

Phase 6b prepares the backend for a Web frontend: state frames now support full/lite modes, every reset creates a new `run_id`, event injection is observable as queued then handled, and CORS is enabled for local Vite/dev-server origins.

## Start

```bash
python -m uvicorn uav_search.server.app:app --reload
```

## HTTP Examples

```bash
curl http://127.0.0.1:8000/api/health
curl http://127.0.0.1:8000/api/scenarios
```

```bash
curl -X POST http://127.0.0.1:8000/api/sim/reset ^
  -H "Content-Type: application/json" ^
  -d "{\"config_path\":\"config/default.yaml\",\"scenario_path\":\"config/scenarios/area_search_2uav_target_confirm.yaml\"}"
```

```bash
curl -X POST http://127.0.0.1:8000/api/sim/step ^
  -H "Content-Type: application/json" ^
  -d "{\"steps\":1}"
```

```bash
curl -X POST http://127.0.0.1:8000/api/sim/event ^
  -H "Content-Type: application/json" ^
  -d "{\"type\":\"TARGET_FOUND\",\"source_uav_id\":null,\"data\":{\"target_id\":\"web_target_001\",\"position\":{\"x\":20,\"y\":20},\"confidence\":0.85,\"target_type\":\"unknown\",\"orbit_radius_cells\":2,\"orbit_laps\":1,\"dwell_s\":5}}"
```

```bash
curl -X POST http://127.0.0.1:8000/api/sim/event ^
  -H "Content-Type: application/json" ^
  -d "{\"type\":\"MAP_UPDATE\",\"data\":{\"operation\":\"add_obstacle\",\"x\":20,\"y\":20,\"width\":5,\"height\":5}}"
```

```bash
curl "http://127.0.0.1:8000/api/sim/state?include_map=false&state_level=lite"
```

## Endpoints

- `GET /api/health`
- `GET /api/scenarios`
- `POST /api/sim/reset`
- `POST /api/sim/step`
- `POST /api/sim/start`
- `POST /api/sim/pause`
- `GET /api/sim/state?include_map=true`
- `GET /api/sim/metrics`
- `POST /api/sim/event`
- `WS /ws/sim`

## State Levels

`reset` returns a full state by default. `step`, `start`, `pause`, and WebSocket tick broadcasts return lite state by default.

Lite state contains:

- `time_s`, `tick`, `running`, `run_id`, `scenario_name`
- `global_coverage`, `priority_coverage`
- `uavs`, `commands`, `command_acks`
- `events`, `pending_events`, `recent_events`, `event_log`
- `advisory_summary`, `tasks`, `targets`, `changed_cells`, `coverage_changed_cells`
- `active_commands` with `remaining_path` and progress for current executable commands
- `metrics`

Full state includes all lite fields plus:

- `map.terrain`
- `map.passable`
- `map.coverage_count`
- `map.search_confidence`
- `map.search_priority`

Use `GET /api/sim/state?include_map=true&state_level=full` when the frontend needs a full map refresh. Use `include_map=false&state_level=lite` for frequent polling or timeline panels.

## Run Id

Each `POST /api/sim/reset` creates a new `run_id`. Every HTTP state response and WebSocket state frame includes that value so a frontend can discard stale frames from an old run.

## CORS

The server currently allows local frontend origins:

- `http://localhost:5173`
- `http://127.0.0.1:5173`
- `http://localhost:3000`
- `http://127.0.0.1:3000`

## Event Flow

`POST /api/sim/event` returns:

```json
{
  "event_id": "server_target_found_...",
  "queued": true,
  "state": { "...": "lite state" }
}
```

The event appears in `pending_events` immediately. After the next tick handles it, the event moves into `recent_events` and is appended to `event_log`. Scenario-injected events also appear in `recent_events` after they are handled.

For `MAP_UPDATE`, the next state includes `changed_cells`. Lite frames carry those changed cells; the frontend can request a full map with `GET /api/sim/state?include_map=true&state_level=full` when it needs to resync the complete grid.

## WebSocket

`WS /ws/sim` sends one full state frame immediately after connection. Later broadcasts from `step`, `start`, and event handling are lite frames by default. A reset broadcasts a full state because the map, task state, and `run_id` may all have changed.

Slow or disconnected WebSocket clients are removed from the connection set and do not stop the simulation runtime.
