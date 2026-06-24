from __future__ import annotations

from typing import Any

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from uav_search.server.runtime import SimulationRuntime
from uav_search.server.schemas import EventRequest, ResetRequest, StartRequest, StepRequest


class WebSocketManager:
    def __init__(self) -> None:
        self._connections: set[WebSocket] = set()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self._connections.add(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        self._connections.discard(websocket)

    async def broadcast(self, state: dict[str, Any]) -> None:
        for websocket in list(self._connections):
            try:
                await websocket.send_json(state)
            except Exception:
                self.disconnect(websocket)


runtime = SimulationRuntime()
ws_manager = WebSocketManager()
runtime.set_state_callback(ws_manager.broadcast)

app = FastAPI(title="UAV Search Simulation Server")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "http://127.0.0.1:5173",
        "http://localhost:3000",
        "http://127.0.0.1:3000",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health() -> dict[str, Any]:
    return {"status": "ok", "running": runtime.running}


@app.get("/api/scenarios")
def scenarios() -> dict[str, Any]:
    return {"scenarios": runtime.get_scenarios()}


@app.post("/api/sim/reset")
async def reset(request: ResetRequest) -> dict[str, Any]:
    state = _call_runtime(runtime.reset, request.config_path, request.scenario_path)
    await ws_manager.broadcast(state)
    return state


@app.post("/api/sim/step")
async def step(request: StepRequest) -> dict[str, Any]:
    state = _call_runtime(runtime.step, request.steps)
    await ws_manager.broadcast(state)
    return state


@app.post("/api/sim/start")
async def start(request: StartRequest | None = None) -> dict[str, Any]:
    state = _call_runtime(runtime.start, (request or StartRequest()).tick_interval_ms)
    await ws_manager.broadcast(state)
    return state


@app.post("/api/sim/pause")
async def pause() -> dict[str, Any]:
    state = _call_runtime(runtime.pause)
    await ws_manager.broadcast(state)
    return state


@app.get("/api/sim/state")
def state(
    include_map: bool = Query(default=True),
    state_level: str = Query(default="full", pattern="^(full|lite)$"),
) -> dict[str, Any]:
    return _call_runtime(runtime.get_state, include_map=include_map, state_level=state_level)


@app.get("/api/sim/metrics")
def metrics() -> dict[str, Any]:
    return _call_runtime(runtime.get_metrics)


@app.post("/api/sim/event")
async def enqueue_event(request: EventRequest) -> dict[str, Any]:
    payload = _call_runtime(runtime.enqueue_event, request)
    await ws_manager.broadcast(payload["state"])
    return payload


@app.websocket("/ws/sim")
async def websocket_sim(websocket: WebSocket) -> None:
    await ws_manager.connect(websocket)
    try:
        await websocket.send_json(_call_runtime(runtime.get_state, include_map=True, state_level="full"))
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)
    except Exception:
        ws_manager.disconnect(websocket)


def _call_runtime(func, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail="simulation runtime error") from exc
