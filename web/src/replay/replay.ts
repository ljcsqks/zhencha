import type { CommandAckSnapshot, ControlCommandSnapshot, EventRecord, SimulationMap, SimulationState, UavState } from "../types/sim";

export interface ReplayStep {
  time_s: number;
  global_coverage?: number;
  priority_coverage?: number;
  uavs?: UavState[];
  commands?: ControlCommandSnapshot[];
  command_acks?: CommandAckSnapshot[];
  events?: string[];
  changed_cells?: Array<{ x: number; y: number }>;
  coverage_changed_cells?: Array<{ x: number; y: number; coverage_count: number; search_confidence?: number }>;
  active_commands?: SimulationState["active_commands"];
  tasks?: SimulationState["tasks"];
  target_metrics?: SimulationState["targets"];
  targets?: SimulationState["targets"];
  advisory_summary?: Record<string, unknown>;
  map?: SimulationMap;
}

export interface ReplayPayload {
  run_id: string;
  scenario_name: string;
  summary: Record<string, unknown>;
  steps: ReplayStep[];
  map?: SimulationMap;
}

export function parseReplayPayload(payload: unknown): ReplayPayload {
  if (!isRecord(payload)) {
    throw new Error("Replay file must be a JSON object");
  }
  const summary = isRecord(payload.summary) ? payload.summary : {};
  const steps = Array.isArray(payload.steps) ? payload.steps : [];
  if (steps.length === 0) {
    throw new Error("Replay file does not contain steps");
  }
  return {
    run_id: String(payload.run_id || "unknown_run"),
    scenario_name: String(payload.scenario_name || summary.scenario_name || "replay"),
    summary,
    steps: steps.map((step) => (isRecord(step) ? step as unknown as ReplayStep : { time_s: 0 })),
    map: isRecord(payload.map) ? payload.map as unknown as SimulationMap : undefined,
  };
}

export function replayStepToState(
  step: ReplayStep,
  runId: string,
  scenarioName: string,
  tick: number,
  replayMap?: SimulationMap,
): SimulationState {
  return {
    time_s: Number(step.time_s || tick),
    tick,
    running: false,
    run_id: `replay_${runId}`,
    scenario_name: scenarioName,
    global_coverage: Number(step.global_coverage || 0),
    priority_coverage: Number(step.priority_coverage || 0),
    uavs: step.uavs || [],
    commands: step.commands || [],
    command_acks: step.command_acks || [],
    events: step.events || [],
    pending_events: [],
    recent_events: replayEvents(step.events || []),
    event_log: replayEvents(step.events || []),
    advisory_summary: step.advisory_summary || {},
    tasks: step.tasks || {},
    targets: step.targets || step.target_metrics || {},
    changed_cells: step.changed_cells || [],
    coverage_changed_cells: step.coverage_changed_cells || [],
    active_commands: step.active_commands || [],
    metrics: {
      global_coverage: Number(step.global_coverage || 0),
      priority_coverage: Number(step.priority_coverage || 0),
    },
    map: step.map || replayMap,
  };
}

function replayEvents(events: string[]): EventRecord[] {
  return events.map((eventId) => ({
    event_id: eventId,
    type: "REPLAY_EVENT",
    status: "handled",
  }));
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}
