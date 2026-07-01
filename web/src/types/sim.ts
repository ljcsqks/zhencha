export type StateLevel = "full" | "lite";

export interface GridPosition {
  x: number;
  y: number;
}

export interface SimulationMap {
  width_cells: number;
  height_cells: number;
  resolution_m: number;
  terrain: string[][];
  passable: boolean[][];
  coverage_count: number[][];
  search_confidence: number[][];
  search_priority: number[][];
}

export interface UavState {
  id: string;
  position: GridPosition;
  status: string;
  battery: number;
  home_position?: GridPosition;
  sensor_radius_cells?: number;
  speed_mps?: number;
  task_id?: string | null;
  total_distance_m?: number;
  effective_search_distance_m?: number;
  idle_reason?: string | null;
}

export interface DraftRectangle {
  id?: string;
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface DraftPriorityRegion extends DraftRectangle {
  priority: number;
}

export interface DraftMapConfig {
  width_cells: number;
  height_cells: number;
  resolution_m: number;
}

export interface DraftUav {
  id: string;
  home_position: GridPosition;
  initial_position: GridPosition;
  sensor_radius_cells: number;
  speed_mps: number;
  battery: number;
}

export interface MissionDraft {
  draftUavs: DraftUav[];
  draftObstacles: DraftRectangle[];
  draftSearchRegion: DraftRectangle | null;
  draftPriorityRegions: DraftPriorityRegion[];
  draftMapConfig: DraftMapConfig | null;
}

export interface ControlCommandSnapshot {
  time_s?: number;
  command_id: string;
  command: string;
  uav_id: string | null;
  task_id?: string | null;
  target?: unknown;
  path?: GridPosition[];
  reason?: string | null;
  executable?: boolean;
  advisory?: boolean;
  issued_at?: number | null;
  updated_at?: number | null;
  metadata?: Record<string, unknown>;
}

export interface CommandAckSnapshot {
  command_id: string;
  uav_id?: string | null;
  status: string;
  reason?: string | null;
  progress?: number | null;
  issued_at?: number | null;
  updated_at?: number | null;
}

export interface EventRecord {
  event_id: string;
  type: string;
  status?: string;
  queued_at_s?: number | null;
  handled_at_s?: number | null;
  source_uav_id?: string | null;
  data?: Record<string, unknown>;
  source?: string;
}

export interface ScenarioInfo {
  name: string;
  path: string;
  description?: string;
}

export interface AlgorithmInfo {
  version: string;
  label: string;
  description: string;
}

export interface SimulationState {
  time_s: number;
  tick: number;
  running: boolean;
  run_id: string;
  scenario_name: string;
  algorithm_version?: string;
  available_algorithm_versions?: string[];
  global_coverage: number;
  priority_coverage: number;
  uavs: UavState[];
  commands: ControlCommandSnapshot[];
  command_acks: CommandAckSnapshot[];
  events: string[];
  pending_events: EventRecord[];
  recent_events: EventRecord[];
  event_log: EventRecord[];
  advisory_summary: Record<string, unknown>;
  tasks: Record<string, unknown>;
  targets: Record<string, unknown>;
  diagnostics?: Record<string, unknown>;
  changed_cells: GridPosition[];
  coverage_changed_cells?: Array<GridPosition & { coverage_count: number; search_confidence?: number }>;
  active_commands?: ActiveCommandSnapshot[];
  metrics: Record<string, unknown>;
  map?: SimulationMap;
}

export interface ActiveCommandSnapshot {
  uav_id: string;
  command_id: string;
  command: string;
  task_id?: string | null;
  path: GridPosition[];
  remaining_path: GridPosition[];
  progress?: number | null;
  issued_at?: number | null;
  metadata?: Record<string, unknown>;
}

export interface EventRequest {
  type: "TARGET_FOUND" | "MAP_UPDATE" | "UAV_OFFLINE" | "UAV_RECOVERED" | "BUILDING_MODEL_REQUEST";
  time_s?: number | null;
  source_uav_id?: string | null;
  data: Record<string, unknown>;
}

export interface ScenarioListResponse {
  scenarios: ScenarioInfo[];
}

export interface AlgorithmListResponse {
  algorithms: AlgorithmInfo[];
  default_version: string;
}

export interface EventResponse {
  event_id: string;
  queued: boolean;
  state: SimulationState;
}

export interface ExportResponse {
  run_id: string;
  export_dir: string;
  files: string[];
}
