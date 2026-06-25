# Demo Runbook

This runbook shows how to demonstrate the UAV search simulation console added through Phase 7c.

## Environment

Start the backend:

```bash
python -m uvicorn uav_search.server.app:app --reload
```

Start the frontend:

```bash
cd web
npm install
npm run dev
```

Open `http://127.0.0.1:5173`.

## Demo 1: Multi-UAV Search

Select `demo_search_3uav` from the Demo panel.

1. Click `Reset`.
2. Click `Start` and let the simulation run for a few seconds.
3. Click `Pause`, then use `Step N` for controlled advancement.

Watch:

- UAV trajectories and active planned paths on the map.
- `global_coverage`, `priority_coverage`, and `total_distance_m`.
- Acceptance panel rows for coverage, priority coverage, and no-fly violations.

## Demo 2: Target Confirmation

Select `demo_target_confirm`.

1. Click `Reset`.
2. Choose `Inject Target`.
3. Click a reachable cell near the map interior.
4. Use `Step N` until `CONFIRM_TARGET` appears and receives a `completed` ack.

Watch:

- Command Log: `CONFIRM_TARGET`, latest ack status, progress, and reason.
- Target panel: target success state.
- Metrics: `confirm_success_rate`, `target_response_time_s`, and `interrupted_task_resume_rate`.
- UAV panel: the confirming UAV returns to normal search after confirmation.

## Demo 3: Dynamic Obstacle

Select `demo_dynamic_obstacle`.

1. Click `Reset`.
2. Choose `Add Obstacle`.
3. Drag a rectangle on the map.
4. Step the simulation once.

Watch:

- Event Log: queued then handled `MAP_UPDATE`.
- Map: changed cells highlighted and full map refresh after terrain changes.
- Command Log: replanning-related command/ack changes.
- Acceptance: `no_fly_violations` should remain PASS.

## Demo 4: UAV Offline And Recover

Select `demo_uav_offline_recover`.

1. Click `Reset`.
2. In the UAV panel, click `Offline` for one UAV.
3. Step the simulation and watch failed/cancelled command acks.
4. Click `Recover` for that UAV.
5. Continue stepping.

Watch:

- UAV status changes between `OFFLINE` and active states.
- Command Log: failed or cancelled acks while the UAV is offline.
- Task and Acceptance panels for recovery behavior.

## Export Run

After any run has at least one snapshot:

1. Click `Export Run`.
2. The page shows the server export directory and file list.

Exports are written under:

```text
runs/web_exports/<run_id>/
```

Files:

- `snapshots.json`: replay-compatible timeline.
- `metrics.json`: full metrics result.
- `final_state.json`: final web state with map.
- `event_log.json`: queued/handled event records.
- `command_log.json`: command and ack timeline.
- `scenario.yaml`: copied scenario.
- `config.yaml`: copied base config.
- `summary.json`: compact demo summary.

## Replay Run

1. In the Replay panel, click `Load snapshots.json`.
2. Select an exported `snapshots.json`.
3. Drag the slider to inspect a tick.
4. Use `Play replay` / `Pause replay`.
5. Change playback speed as needed.
6. Click `Exit replay` to return to live simulation controls.

Replay mode is clearly marked as `Replay`. Real-time start, event injection, obstacle editing, and UAV offline/recover controls are hidden or disabled while replay is active.

## Known Limits

- Single backend runtime only.
- This is not a real flight controller.
- No target strike or attack logic.
- No Docker in this phase.
- No 3D or GIS view.
- The current search algorithm still has known 5-UAV efficiency issues; Phase 7c does not optimize it.
