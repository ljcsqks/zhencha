import { Pause, Play, Upload } from "lucide-react";
import { useEffect, useMemo, useState } from "react";
import { parseReplayPayload, replayStepToState, type ReplayPayload } from "../replay/replay";
import type { SimulationMap, SimulationState } from "../types/sim";

interface Props {
  active: boolean;
  onReplayState(state: SimulationState | undefined): void;
  onExit(): void;
}

export function ReplayPanel({ active, onReplayState, onExit }: Props) {
  const [replay, setReplay] = useState<ReplayPayload>();
  const [tick, setTick] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speedMs, setSpeedMs] = useState(250);
  const [error, setError] = useState<string>();

  const maxTick = Math.max(0, (replay?.steps.length || 1) - 1);
  const replayMap = useMemo(() => replay?.map ? buildReplayMap(replay.map, replay.steps, tick) : undefined, [replay, tick]);

  useEffect(() => {
    if (!replay) {
      onReplayState(undefined);
      return;
    }
    onReplayState(replayStepToState(replay.steps[tick], replay.run_id, replay.scenario_name, tick, replayMap));
  }, [onReplayState, replay, replayMap, tick]);

  useEffect(() => {
    if (!playing || !replay) {
      return undefined;
    }
    const timer = window.setInterval(() => {
      setTick((current) => {
        if (current >= maxTick) {
          setPlaying(false);
          return current;
        }
        return current + 1;
      });
    }, speedMs);
    return () => window.clearInterval(timer);
  }, [maxTick, playing, replay, speedMs]);

  const loadFile = async (file: File | undefined) => {
    if (!file) {
      return;
    }
    try {
      setError(undefined);
      const payload = JSON.parse(await file.text());
      const parsed = parseReplayPayload(payload);
      setReplay(parsed);
      setTick(0);
      setPlaying(false);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  };

  const exitReplay = () => {
    setReplay(undefined);
    setPlaying(false);
    setTick(0);
    onExit();
  };

  return (
    <section className={`panel replay-panel ${active ? "active" : ""}`}>
      <div className="panel-heading">
        <h2>Replay</h2>
        {active && <span className="status-pill idle">Replay mode</span>}
      </div>
      <label className="file-picker">
        <Upload size={15} /> Load snapshots.json
        <input type="file" accept="application/json,.json" onChange={(event) => loadFile(event.target.files?.[0])} />
      </label>
      {error && <small className="error-text">{error}</small>}
      {replay && (
        <>
          <div className="replay-meta">
            <strong>{replay.scenario_name}</strong>
            <span className="mono compact">{replay.run_id}</span>
          </div>
          <input
            type="range"
            min={0}
            max={maxTick}
            value={tick}
            onChange={(event) => setTick(Number(event.target.value))}
          />
          <div className="inline-control">
            <button onClick={() => setPlaying((value) => !value)}>
              {playing ? <Pause size={15} /> : <Play size={15} />}
              {playing ? "Pause replay" : "Play replay"}
            </button>
            <select value={speedMs} onChange={(event) => setSpeedMs(Number(event.target.value))}>
              <option value={500}>0.5x</option>
              <option value={250}>1x</option>
              <option value={100}>2.5x</option>
            </select>
          </div>
          <small>
            tick {tick} / {maxTick}
          </small>
          <button className="wide-button" onClick={exitReplay}>Exit replay</button>
        </>
      )}
      {!replay && <small>Load an exported snapshots.json file to inspect a completed run.</small>}
    </section>
  );
}

function buildReplayMap(base: SimulationMap, steps: ReplayPayload["steps"], tick: number): SimulationMap {
  const map: SimulationMap = {
    ...base,
    coverage_count: base.coverage_count.map((row) => row.map(() => 0)),
    search_confidence: base.search_confidence.map((row) => row.map(() => 0)),
  };
  for (const step of steps.slice(0, tick + 1)) {
    for (const cell of step.coverage_changed_cells || []) {
      if (map.coverage_count[cell.y]?.[cell.x] !== undefined) {
        map.coverage_count[cell.y][cell.x] = cell.coverage_count;
      }
      if (typeof cell.search_confidence === "number" && map.search_confidence[cell.y]?.[cell.x] !== undefined) {
        map.search_confidence[cell.y][cell.x] = cell.search_confidence;
      }
    }
  }
  return map;
}
