import type {
  CommandAckSnapshot,
  ControlCommandSnapshot,
  EventRequest,
  EventRecord,
  GridPosition,
  SimulationState,
} from "../types/sim";

export interface CommandLogEntry extends ControlCommandSnapshot {
  ack_status?: string;
  progress?: number | null;
  updated_at?: number | null;
}

export interface SimulationClientState {
  currentState?: SimulationState;
  fullMapState?: SimulationState;
  runId?: string;
  commandLog: CommandLogEntry[];
  commandById: Record<string, ControlCommandSnapshot>;
  ackByCommandId: Record<string, CommandAckSnapshot>;
  commandOrder: string[];
  eventLog: EventRecord[];
  eventById: Record<string, EventRecord>;
  eventOrder: string[];
  uavTrajectories: Record<string, GridPosition[]>;
  needsFullMapRefresh: boolean;
}

const LOG_LIMIT = 300;
const TRAJECTORY_LIMIT = 1200;

export function emptySimulationClientState(): SimulationClientState {
  return {
    commandLog: [],
    commandById: {},
    ackByCommandId: {},
    commandOrder: [],
    eventLog: [],
    eventById: {},
    eventOrder: [],
    uavTrajectories: {},
    needsFullMapRefresh: false,
  };
}

export function mergeSimulationState(
  previous: SimulationClientState | undefined,
  incoming: SimulationState,
): SimulationClientState {
  const base = previous ?? emptySimulationClientState();
  const runChanged = Boolean(base.runId && base.runId !== incoming.run_id);
  const fullMapState = incoming.map
    ? incoming
    : runChanged
      ? undefined
      : applyCoveragePatch(base.fullMapState, incoming.coverage_changed_cells || []);
  const previousTrajectories = runChanged ? {} : base.uavTrajectories;
  const commandState = mergeCommands(runChanged ? emptySimulationClientState() : base, incoming);
  const eventState = mergeEvents(runChanged ? emptySimulationClientState() : base, incoming);

  return {
    currentState: incoming,
    fullMapState,
    runId: incoming.run_id,
    commandLog: commandState.commandLog,
    commandById: commandState.commandById,
    ackByCommandId: commandState.ackByCommandId,
    commandOrder: commandState.commandOrder,
    eventLog: eventState.eventLog,
    eventById: eventState.eventById,
    eventOrder: eventState.eventOrder,
    uavTrajectories: appendTrajectories(previousTrajectories, incoming),
    needsFullMapRefresh: !incoming.map && (incoming.changed_cells || []).length > 0,
  };
}

export function shouldStepAfterEvent(state?: Pick<SimulationState, "running">): boolean {
  return !state?.running;
}

export function shouldRefreshFullStateAfterEvent(
  event: Pick<EventRequest, "type">,
  state: Pick<SimulationState, "changed_cells">,
): boolean {
  return (
    event.type === "MAP_UPDATE" ||
    event.type === "UAV_OFFLINE" ||
    event.type === "UAV_RECOVERED" ||
    (state.changed_cells || []).length > 0
  );
}

function mergeCommands(base: SimulationClientState, incoming: SimulationState) {
  const commandById: Record<string, ControlCommandSnapshot> = { ...(base.commandById || commandMapFromLog(base.commandLog)) };
  const ackByCommandId: Record<string, CommandAckSnapshot> = { ...(base.ackByCommandId || {}) };
  let commandOrder = [...(base.commandOrder?.length ? base.commandOrder : base.commandLog.map((entry) => entry.command_id))];
  for (const command of incoming.commands || []) {
    commandById[command.command_id] = command;
    commandOrder = touch(commandOrder, command.command_id);
  }
  for (const ack of incoming.command_acks || []) {
    ackByCommandId[ack.command_id] = ack;
    commandOrder = touch(commandOrder, ack.command_id);
  }
  commandOrder = commandOrder.slice(-LOG_LIMIT);
  pruneToKeys(commandById, commandOrder);
  pruneToKeys(ackByCommandId, commandOrder);
  const commandLog = commandOrder.map((commandId) => {
    const command = commandById[commandId] || ackOnlyCommand(commandId, ackByCommandId[commandId]);
    return mergeCommandAndAck(command, ackByCommandId[commandId]);
  });
  return { commandLog, commandById, ackByCommandId, commandOrder };
}

function mergeCommandAndAck(command: ControlCommandSnapshot, ack?: CommandAckSnapshot): CommandLogEntry {
  return {
    ...command,
    ack_status: ack?.status,
    progress: ack?.progress,
    updated_at: ack?.updated_at ?? command.time_s,
    reason: ack?.reason ?? command.reason,
  };
}

function ackOnlyCommand(commandId: string, ack?: CommandAckSnapshot): ControlCommandSnapshot {
  return {
    command_id: commandId,
    command: "ACK_ONLY",
    uav_id: ack?.uav_id ?? null,
    task_id: null,
    executable: false,
    advisory: false,
    path: [],
    reason: ack?.reason ?? null,
  };
}

function commandMapFromLog(log: CommandLogEntry[]): Record<string, ControlCommandSnapshot> {
  return Object.fromEntries(log.map((entry) => [entry.command_id, entry]));
}

function mergeEvents(base: SimulationClientState, incoming: SimulationState) {
  const eventById: Record<string, EventRecord> = { ...(base.eventById || Object.fromEntries(base.eventLog.map((event) => [event.event_id, event]))) };
  let eventOrder = [...(base.eventOrder?.length ? base.eventOrder : base.eventLog.map((event) => event.event_id))];
  const incomingEvents = [
    ...(incoming.pending_events || []),
    ...(incoming.recent_events || []),
    ...(incoming.event_log || []),
  ];
  for (const event of incomingEvents) {
    const previous = eventById[event.event_id];
    eventById[event.event_id] = preferHandled(previous, event);
    eventOrder = touch(eventOrder, event.event_id);
  }
  eventOrder = eventOrder.slice(-LOG_LIMIT);
  pruneToKeys(eventById, eventOrder);
  return {
    eventLog: eventOrder.map((eventId) => eventById[eventId]).filter(Boolean),
    eventById,
    eventOrder,
  };
}

function preferHandled(previous: EventRecord | undefined, next: EventRecord): EventRecord {
  if (!previous) {
    return next;
  }
  if (previous.status !== "handled" && next.status === "handled") {
    return { ...previous, ...next };
  }
  if (previous.status === "handled" && next.status !== "handled") {
    return previous;
  }
  return { ...previous, ...next };
}

function applyCoveragePatch(
  fullMapState: SimulationState | undefined,
  patch: Array<GridPosition & { coverage_count: number; search_confidence?: number }>,
): SimulationState | undefined {
  if (!fullMapState?.map || patch.length === 0) {
    return fullMapState;
  }
  const map = {
    ...fullMapState.map,
    coverage_count: fullMapState.map.coverage_count.map((row) => [...row]),
    search_confidence: fullMapState.map.search_confidence.map((row) => [...row]),
  };
  for (const cell of patch) {
    if (map.coverage_count[cell.y]?.[cell.x] !== undefined) {
      map.coverage_count[cell.y][cell.x] = cell.coverage_count;
    }
    if (typeof cell.search_confidence === "number" && map.search_confidence[cell.y]?.[cell.x] !== undefined) {
      map.search_confidence[cell.y][cell.x] = cell.search_confidence;
    }
  }
  return { ...fullMapState, map };
}

function appendTrajectories(
  previous: Record<string, GridPosition[]>,
  incoming: SimulationState,
): Record<string, GridPosition[]> {
  const next: Record<string, GridPosition[]> = { ...previous };
  for (const uav of incoming.uavs || []) {
    const existing = next[uav.id] || [];
    const last = existing.at(-1);
    if (!last || last.x !== uav.position.x || last.y !== uav.position.y) {
      next[uav.id] = [...existing, uav.position].slice(-TRAJECTORY_LIMIT);
    }
  }
  return next;
}

function touch(items: string[], item: string): string[] {
  return [...items.filter((existing) => existing !== item), item];
}

function pruneToKeys<T>(record: Record<string, T>, keys: string[]): void {
  const allowed = new Set(keys);
  for (const key of Object.keys(record)) {
    if (!allowed.has(key)) {
      delete record[key];
    }
  }
}
