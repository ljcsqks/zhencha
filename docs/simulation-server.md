# Simulation Server

Phase 6 adds an in-process FastAPI service over the existing simulation runtime. The service does not call scheduler, map, or fleet internals directly; HTTP and WebSocket handlers operate through `SimulationRuntime`.

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

The WebSocket sends full state frames. The first implementation prioritizes debuggability over incremental updates.
