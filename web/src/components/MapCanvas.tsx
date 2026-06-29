import { useEffect, useMemo, useRef, useState } from "react";
import type { UseSimulationResult } from "../hooks/useSimulation";
import type { ActiveCommandSnapshot, ControlCommandSnapshot, DraftUav, GridPosition, SimulationMap } from "../types/sim";

interface Props {
  sim: UseSimulationResult;
}

interface Viewport {
  scale: number;
  offsetX: number;
  offsetY: number;
}

type PointerMode = "pan" | "dragUav" | "selectObstacle";

const UAV_COLORS = ["#0077b6", "#e76f00", "#198754", "#d64562", "#7b2cbf", "#008c7a"];

export function MapCanvas({ sim }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [hover, setHover] = useState<GridPosition | null>(null);
  const [dragStart, setDragStart] = useState<GridPosition | null>(null);
  const [dragEnd, setDragEnd] = useState<GridPosition | null>(null);
  const [viewport, setViewport] = useState<Viewport>({ scale: 1, offsetX: 0, offsetY: 0 });
  const pointerRef = useRef<{ mode: PointerMode; startClient: GridPosition; uavId?: string } | null>(null);
  const state = sim.currentState;
  const mapState = sim.fullMapState?.map;
  const commands = state?.commands || [];
  const activeCommands = state?.active_commands || [];

  const plannedPaths = useMemo(
    () =>
      activeCommands.length > 0
        ? activeCommands.filter((command) => (command.remaining_path || []).length > 0)
        : commands.filter((command) => (command.path || []).length > 0),
    [activeCommands, commands],
  );

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !mapState || !state) {
      return;
    }
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = Math.max(1, Math.floor(rect.width * dpr));
    canvas.height = Math.max(1, Math.floor(rect.height * dpr));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    drawMap(ctx, rect.width, rect.height, mapState, viewport, sim.showCoverage, sim.showGrid, state.changed_cells || []);
    if (sim.showPlannedPath) {
      drawPlannedPaths(ctx, rect.width, rect.height, mapState, viewport, plannedPaths, sim.selectedCommandId, sim.selectedUavId);
    }
    if (sim.showHistoryPath) {
      drawTrajectories(ctx, rect.width, rect.height, mapState, viewport, sim.uavTrajectories, sim.selectedUavId);
    }
    drawDraftUavs(ctx, rect.width, rect.height, mapState, viewport, sim.missionDraft.draftUavs, sim.selectedUavId, sim.draftEditable);
    drawUavs(ctx, rect.width, rect.height, mapState, viewport, state.uavs || [], sim.selectedUavId);
    if (dragStart && dragEnd) {
      drawSelection(ctx, rect.width, rect.height, mapState, viewport, dragStart, dragEnd);
    }
  }, [
    dragEnd,
    dragStart,
    mapState,
    plannedPaths,
    sim.draftEditable,
    sim.missionDraft.draftUavs,
    sim.showCoverage,
    sim.showGrid,
    sim.showHistoryPath,
    sim.showPlannedPath,
    sim.selectedCommandId,
    sim.selectedUavId,
    sim.uavTrajectories,
    state,
    viewport,
  ]);

  const gridAtPointer = (event: React.PointerEvent<HTMLCanvasElement>): GridPosition | null => {
    if (!mapState || !canvasRef.current) {
      return null;
    }
    const rect = canvasRef.current.getBoundingClientRect();
    return screenToGrid(mapState, rect.width, rect.height, viewport, event.clientX - rect.left, event.clientY - rect.top);
  };

  const clientPoint = (event: React.PointerEvent<HTMLCanvasElement>): GridPosition => ({ x: event.clientX, y: event.clientY });

  const finishObstacleDrag = async () => {
    if (!dragStart || !dragEnd) {
      setDragStart(null);
      setDragEnd(null);
      return;
    }
    if (sim.toolMode !== "addObstacle" && sim.toolMode !== "removeObstacle") {
      setDragStart(null);
      setDragEnd(null);
      return;
    }
    const x = Math.min(dragStart.x, dragEnd.x);
    const y = Math.min(dragStart.y, dragEnd.y);
    const width = Math.abs(dragStart.x - dragEnd.x) + 1;
    const height = Math.abs(dragStart.y - dragEnd.y) + 1;
    await sim.updateObstacle(sim.toolMode === "addObstacle" ? "add_obstacle" : "remove_obstacle", x, y, width, height);
    setDragStart(null);
    setDragEnd(null);
  };

  const hoverInfo = hover && mapState ? cellInfo(mapState, hover) : "";
  const editableHint = sim.draftEditable ? "draft editing enabled" : "draft locked while running";

  return (
    <section className="map-card">
      <div className="map-head">
        <div>
          <h2>Mission Map</h2>
          <span>{mapState ? `${mapState.width_cells} x ${mapState.height_cells} cells / ${editableHint}` : "reset to load map"}</span>
        </div>
        <div className="map-actions">
          <button onClick={() => setViewport({ scale: 1, offsetX: 0, offsetY: 0 })}>Center view</button>
          <span className="mode-chip">{sim.toolMode}</span>
        </div>
      </div>
      <canvas
        ref={canvasRef}
        className={`map-canvas ${sim.toolMode === "addUav" ? "add-mode" : ""}`}
        onWheel={(event) => {
          if (!mapState || !canvasRef.current) return;
          event.preventDefault();
          const rect = canvasRef.current.getBoundingClientRect();
          const mouseX = event.clientX - rect.left;
          const mouseY = event.clientY - rect.top;
          setViewport((current) => zoomViewport(mapState, rect.width, rect.height, current, mouseX, mouseY, event.deltaY < 0 ? 1.12 : 0.88));
        }}
        onPointerMove={(event) => {
          const point = gridAtPointer(event);
          setHover(point);
          const active = pointerRef.current;
          if (!active) return;
          if (active.mode === "pan") {
            const next = clientPoint(event);
            setViewport((current) => ({
              ...current,
              offsetX: current.offsetX + next.x - active.startClient.x,
              offsetY: current.offsetY + next.y - active.startClient.y,
            }));
            pointerRef.current = { ...active, startClient: next };
          } else if (active.mode === "dragUav" && point && active.uavId) {
            sim.moveDraftUavTo(active.uavId, point);
          } else if (active.mode === "selectObstacle" && point) {
            setDragEnd(point);
          }
        }}
        onPointerDown={(event) => {
          const point = gridAtPointer(event);
          if (!point) {
            return;
          }
          if (sim.toolMode === "addUav" && sim.draftEditable) {
            sim.addDraftUavAt(point);
            return;
          }
          const draftHit = hitDraftUav(point, sim.missionDraft.draftUavs);
          if (sim.draftEditable && draftHit) {
            sim.setSelectedUavId(draftHit.id);
            pointerRef.current = { mode: "dragUav", startClient: clientPoint(event), uavId: draftHit.id };
            event.currentTarget.setPointerCapture(event.pointerId);
            return;
          }
          if (sim.toolMode === "target") {
            sim.injectTarget(point.x, point.y);
            return;
          }
          if (sim.toolMode === "addObstacle" || sim.toolMode === "removeObstacle") {
            setDragStart(point);
            setDragEnd(point);
            pointerRef.current = { mode: "selectObstacle", startClient: clientPoint(event) };
            event.currentTarget.setPointerCapture(event.pointerId);
            return;
          }
          pointerRef.current = { mode: "pan", startClient: clientPoint(event) };
          event.currentTarget.setPointerCapture(event.pointerId);
        }}
        onPointerUp={(event) => {
          const active = pointerRef.current;
          pointerRef.current = null;
          if (active?.mode === "selectObstacle") {
            finishObstacleDrag();
          }
          try {
            event.currentTarget.releasePointerCapture(event.pointerId);
          } catch {
            // Pointer capture is best-effort across browsers.
          }
        }}
        onPointerLeave={() => {
          setHover(null);
          if (pointerRef.current?.mode === "selectObstacle") {
            finishObstacleDrag();
          }
          pointerRef.current = null;
        }}
      />
      <div className="map-footer">
        <span>{hover ? `x=${hover.x}, y=${hover.y}` : "hover a cell"}</span>
        <span>{hoverInfo}</span>
      </div>
      <div className="legend-strip">
        <span><i className="legend obstacle" /> obstacle/no-fly</span>
        <span><i className="legend priority" /> priority</span>
        <span><i className="legend coverage" /> coverage heat</span>
        <span><i className="legend planned" /> planned path</span>
        <span><i className="legend draft" /> draft UAV</span>
        <span><i className="legend searching" /> live UAV</span>
      </div>
    </section>
  );
}

function drawMap(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  map: SimulationMap,
  viewport: Viewport,
  showCoverage: boolean,
  showGrid: boolean,
  changedCells: GridPosition[],
) {
  ctx.fillStyle = "#e5ebe1";
  ctx.fillRect(0, 0, width, height);
  const frame = mapFrame(map, width, height, viewport);
  ctx.fillStyle = "#f7f8f3";
  ctx.fillRect(frame.x, frame.y, frame.w, frame.h);
  const changed = new Set(changedCells.map((cell) => `${cell.x},${cell.y}`));
  for (let y = 0; y < map.height_cells; y += 1) {
    for (let x = 0; x < map.width_cells; x += 1) {
      const rect = cellRect(map, width, height, viewport, x, y);
      if (!map.passable[y]?.[x]) {
        ctx.fillStyle = "#20272d";
        ctx.fillRect(rect.x, rect.y, rect.w, rect.h);
        continue;
      }
      const priority = map.search_priority[y]?.[x] || 1;
      ctx.fillStyle = priority > 1 ? "rgba(247, 181, 56, 0.5)" : "#edf1e9";
      ctx.fillRect(rect.x, rect.y, rect.w, rect.h);
      if (showCoverage && (map.coverage_count[y]?.[x] || 0) > 0) {
        const alpha = Math.min(0.68, 0.16 + (map.coverage_count[y][x] || 0) * 0.07);
        ctx.fillStyle = `rgba(26, 115, 165, ${alpha})`;
        ctx.fillRect(rect.x, rect.y, rect.w, rect.h);
      }
      if (changed.has(`${x},${y}`)) {
        ctx.fillStyle = "rgba(204, 52, 52, 0.72)";
        ctx.fillRect(rect.x, rect.y, rect.w, rect.h);
      }
    }
  }
  if (showGrid && frame.cell >= 6) {
    ctx.strokeStyle = "rgba(32, 39, 45, 0.11)";
    ctx.lineWidth = 1;
    for (let x = 0; x <= map.width_cells; x += 1) {
      ctx.beginPath();
      ctx.moveTo(frame.x + x * frame.cell, frame.y);
      ctx.lineTo(frame.x + x * frame.cell, frame.y + frame.h);
      ctx.stroke();
    }
    for (let y = 0; y <= map.height_cells; y += 1) {
      ctx.beginPath();
      ctx.moveTo(frame.x, frame.y + y * frame.cell);
      ctx.lineTo(frame.x + frame.w, frame.y + y * frame.cell);
      ctx.stroke();
    }
  }
  ctx.strokeStyle = "#59665e";
  ctx.lineWidth = 2;
  ctx.strokeRect(frame.x, frame.y, frame.w, frame.h);
}

function drawTrajectories(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  map: SimulationMap,
  viewport: Viewport,
  trajectories: Record<string, GridPosition[]>,
  selectedUavId?: string,
) {
  Object.entries(trajectories).forEach(([uavId, points], index) => {
    if (points.length < 2) return;
    ctx.strokeStyle = colorFor(index);
    ctx.lineWidth = selectedUavId === uavId ? 4 : 2;
    ctx.globalAlpha = selectedUavId && selectedUavId !== uavId ? 0.22 : 0.72;
    drawPolyline(ctx, width, height, map, viewport, points);
    ctx.globalAlpha = 1;
  });
}

function drawPlannedPaths(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  map: SimulationMap,
  viewport: Viewport,
  commands: Array<ControlCommandSnapshot | ActiveCommandSnapshot>,
  selectedCommandId?: string,
  selectedUavId?: string,
) {
  commands.forEach((command, index) => {
    ctx.strokeStyle = colorFor(index);
    ctx.setLineDash([6, 5]);
    const highlighted = selectedCommandId === command.command_id || selectedUavId === command.uav_id;
    ctx.lineWidth = highlighted ? 4 : 1.8;
    ctx.globalAlpha = selectedCommandId || selectedUavId ? (highlighted ? 0.95 : 0.22) : 0.84;
    drawPolyline(ctx, width, height, map, viewport, "remaining_path" in command ? command.remaining_path || [] : command.path || []);
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  });
}

function drawUavs(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  map: SimulationMap,
  viewport: Viewport,
  uavs: Array<{ id: string; position: GridPosition; status: string }>,
  selectedUavId?: string,
) {
  uavs.forEach((uav, index) => {
    const point = cellCenter(map, width, height, viewport, uav.position);
    ctx.fillStyle = statusColor(uav.status, index);
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = selectedUavId === uav.id ? 4 : 2;
    ctx.beginPath();
    ctx.arc(point.x, point.y, Math.max(6, mapFrame(map, width, height, viewport).cell * 0.46), 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    drawUavLabel(ctx, uav.id, point.x, point.y);
  });
}

function drawDraftUavs(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  map: SimulationMap,
  viewport: Viewport,
  uavs: DraftUav[],
  selectedUavId?: string,
  editable?: boolean,
) {
  uavs.forEach((uav, index) => {
    const point = cellCenter(map, width, height, viewport, uav.initial_position);
    const radius = Math.max(7, mapFrame(map, width, height, viewport).cell * 0.5);
    ctx.fillStyle = editable ? "rgba(255, 255, 255, 0.92)" : "rgba(210, 214, 210, 0.82)";
    ctx.strokeStyle = selectedUavId === uav.id ? "#0f6ea8" : colorFor(index);
    ctx.lineWidth = selectedUavId === uav.id ? 4 : 2;
    ctx.beginPath();
    ctx.rect(point.x - radius, point.y - radius, radius * 2, radius * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = colorFor(index);
    ctx.beginPath();
    ctx.arc(point.x, point.y, radius * 0.42, 0, Math.PI * 2);
    ctx.fill();
    drawUavLabel(ctx, uav.id, point.x, point.y);
  });
}

function drawSelection(ctx: CanvasRenderingContext2D, width: number, height: number, map: SimulationMap, viewport: Viewport, start: GridPosition, end: GridPosition) {
  const startRect = cellRect(map, width, height, viewport, Math.min(start.x, end.x), Math.min(start.y, end.y));
  const endRect = cellRect(map, width, height, viewport, Math.max(start.x, end.x), Math.max(start.y, end.y));
  ctx.fillStyle = "rgba(15, 110, 168, 0.2)";
  ctx.strokeStyle = "#0f6ea8";
  ctx.lineWidth = 2;
  ctx.fillRect(startRect.x, startRect.y, endRect.x + endRect.w - startRect.x, endRect.y + endRect.h - startRect.y);
  ctx.strokeRect(startRect.x, startRect.y, endRect.x + endRect.w - startRect.x, endRect.y + endRect.h - startRect.y);
}

function drawPolyline(ctx: CanvasRenderingContext2D, width: number, height: number, map: SimulationMap, viewport: Viewport, points: GridPosition[]) {
  if (points.length < 2) return;
  ctx.beginPath();
  points.forEach((point, index) => {
    const screen = cellCenter(map, width, height, viewport, point);
    if (index === 0) {
      ctx.moveTo(screen.x, screen.y);
    } else {
      ctx.lineTo(screen.x, screen.y);
    }
  });
  ctx.stroke();
}

function drawUavLabel(ctx: CanvasRenderingContext2D, id: string, x: number, y: number) {
  ctx.fillStyle = "#111820";
  ctx.font = "700 11px Cascadia Mono, Consolas, monospace";
  ctx.fillText(id.replace("uav_", ""), x + 8, y - 8);
}

function mapFrame(map: SimulationMap, width: number, height: number, viewport: Viewport) {
  const baseCell = Math.min(width / map.width_cells, height / map.height_cells);
  const cell = baseCell * viewport.scale;
  const w = map.width_cells * cell;
  const h = map.height_cells * cell;
  return {
    cell,
    x: (width - w) / 2 + viewport.offsetX,
    y: (height - h) / 2 + viewport.offsetY,
    w,
    h,
  };
}

function cellRect(map: SimulationMap, width: number, height: number, viewport: Viewport, x: number, y: number) {
  const frame = mapFrame(map, width, height, viewport);
  return {
    x: frame.x + x * frame.cell,
    y: frame.y + y * frame.cell,
    w: frame.cell,
    h: frame.cell,
  };
}

function cellCenter(map: SimulationMap, width: number, height: number, viewport: Viewport, point: GridPosition): GridPosition {
  const rect = cellRect(map, width, height, viewport, point.x, point.y);
  return { x: rect.x + rect.w / 2, y: rect.y + rect.h / 2 };
}

function screenToGrid(map: SimulationMap, width: number, height: number, viewport: Viewport, x: number, y: number): GridPosition | null {
  const frame = mapFrame(map, width, height, viewport);
  const gridX = Math.floor((x - frame.x) / frame.cell);
  const gridY = Math.floor((y - frame.y) / frame.cell);
  if (gridX < 0 || gridY < 0 || gridX >= map.width_cells || gridY >= map.height_cells) {
    return null;
  }
  return { x: gridX, y: gridY };
}

function zoomViewport(
  map: SimulationMap,
  width: number,
  height: number,
  viewport: Viewport,
  mouseX: number,
  mouseY: number,
  factor: number,
): Viewport {
  const before = mapFrame(map, width, height, viewport);
  const gridX = (mouseX - before.x) / before.cell;
  const gridY = (mouseY - before.y) / before.cell;
  const scale = Math.min(6, Math.max(0.6, viewport.scale * factor));
  const baseCell = Math.min(width / map.width_cells, height / map.height_cells);
  const cell = baseCell * scale;
  const mapW = map.width_cells * cell;
  const mapH = map.height_cells * cell;
  return {
    scale,
    offsetX: mouseX - gridX * cell - (width - mapW) / 2,
    offsetY: mouseY - gridY * cell - (height - mapH) / 2,
  };
}

function hitDraftUav(point: GridPosition, uavs: DraftUav[]): DraftUav | undefined {
  return uavs.find((uav) => Math.abs(uav.initial_position.x - point.x) <= 1 && Math.abs(uav.initial_position.y - point.y) <= 1);
}

function colorFor(index: number): string {
  return UAV_COLORS[index % UAV_COLORS.length];
}

function statusColor(status: string, index: number): string {
  if (status === "OFFLINE") return "#6f7780";
  if (status === "RETURNING") return "#b9562c";
  if (status === "CONFIRMING") return "#7b2cbf";
  if (status === "SEARCHING") return colorFor(index);
  return "#198754";
}

function cellInfo(map: SimulationMap, point: GridPosition): string {
  const passable = map.passable[point.y]?.[point.x] ? "passable" : "blocked";
  const coverage = map.coverage_count[point.y]?.[point.x] ?? 0;
  const priority = map.search_priority[point.y]?.[point.x] ?? 1;
  return `${passable} / coverage ${coverage} / priority ${priority}`;
}
