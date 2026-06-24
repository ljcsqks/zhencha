import { useEffect, useMemo, useRef, useState } from "react";
import type { UseSimulationResult } from "../hooks/useSimulation";
import type { ActiveCommandSnapshot, ControlCommandSnapshot, GridPosition, SimulationMap } from "../types/sim";

interface Props {
  sim: UseSimulationResult;
}

const UAV_COLORS = ["#00a6ff", "#ff8a00", "#50c878", "#ff4d6d", "#9b5de5", "#00b4a6"];

export function MapCanvas({ sim }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [hover, setHover] = useState<GridPosition | null>(null);
  const [dragStart, setDragStart] = useState<GridPosition | null>(null);
  const [dragEnd, setDragEnd] = useState<GridPosition | null>(null);
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
    drawMap(ctx, rect.width, rect.height, mapState, sim.showCoverage, state.changed_cells || []);
    if (sim.showPlannedPath) {
      drawPlannedPaths(ctx, rect.width, rect.height, mapState, plannedPaths, sim.selectedCommandId, sim.selectedUavId);
    }
    if (sim.showHistoryPath) {
      drawTrajectories(ctx, rect.width, rect.height, mapState, sim.uavTrajectories, sim.selectedUavId);
    }
    drawUavs(ctx, rect.width, rect.height, mapState, state.uavs || [], sim.selectedUavId);
    if (dragStart && dragEnd) {
      drawSelection(ctx, rect.width, rect.height, mapState, dragStart, dragEnd);
    }
  }, [
    dragEnd,
    dragStart,
    mapState,
    plannedPaths,
    sim.showCoverage,
    sim.showHistoryPath,
    sim.showPlannedPath,
    sim.selectedCommandId,
    sim.selectedUavId,
    sim.uavTrajectories,
    state,
  ]);

  const gridAtPointer = (event: React.PointerEvent<HTMLCanvasElement>): GridPosition | null => {
    if (!mapState || !canvasRef.current) {
      return null;
    }
    const rect = canvasRef.current.getBoundingClientRect();
    const x = Math.floor(((event.clientX - rect.left) / rect.width) * mapState.width_cells);
    const y = Math.floor(((event.clientY - rect.top) / rect.height) * mapState.height_cells);
    if (x < 0 || y < 0 || x >= mapState.width_cells || y >= mapState.height_cells) {
      return null;
    }
    return { x, y };
  };

  const finishDrag = async () => {
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

  return (
    <section className="map-card">
      <div className="map-head">
        <div>
          <h2>Mission Map</h2>
          <span>{mapState ? `${mapState.width_cells} x ${mapState.height_cells} cells` : "reset to load map"}</span>
        </div>
        <span className="mode-chip">{sim.toolMode}</span>
      </div>
      <canvas
        ref={canvasRef}
        className="map-canvas"
        onPointerMove={(event) => {
          const point = gridAtPointer(event);
          setHover(point);
          if (dragStart && point) {
            setDragEnd(point);
          }
        }}
        onPointerDown={(event) => {
          const point = gridAtPointer(event);
          if (!point) {
            return;
          }
          if (sim.toolMode === "target") {
            sim.injectTarget(point.x, point.y);
            return;
          }
          if (sim.toolMode === "addObstacle" || sim.toolMode === "removeObstacle") {
            setDragStart(point);
            setDragEnd(point);
          }
        }}
        onPointerUp={finishDrag}
        onPointerLeave={() => {
          setHover(null);
          finishDrag();
        }}
      />
      <div className="map-footer">
        <span>{hover ? `x=${hover.x}, y=${hover.y}` : "hover a cell"}</span>
        <span>{hoverInfo}</span>
      </div>
      <div className="legend-strip">
        <span><i className="legend obstacle" /> obstacle/no-fly</span>
        <span><i className="legend priority" /> priority</span>
        <span><i className="legend coverage" /> coverage</span>
        <span><i className="legend changed" /> changed</span>
        <span><i className="legend searching" /> searching</span>
        <span><i className="legend confirming" /> confirming</span>
      </div>
    </section>
  );
}

function drawMap(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  map: SimulationMap,
  showCoverage: boolean,
  changedCells: GridPosition[],
) {
  ctx.fillStyle = "#f3f5f1";
  ctx.fillRect(0, 0, width, height);
  const cw = width / map.width_cells;
  const ch = height / map.height_cells;
  const changed = new Set(changedCells.map((cell) => `${cell.x},${cell.y}`));
  for (let y = 0; y < map.height_cells; y += 1) {
    for (let x = 0; x < map.width_cells; x += 1) {
      if (!map.passable[y]?.[x]) {
        ctx.fillStyle = "#252b31";
        ctx.fillRect(x * cw, y * ch, cw, ch);
        continue;
      }
      const priority = map.search_priority[y]?.[x] || 1;
      ctx.fillStyle = priority > 1 ? "rgba(255, 192, 75, 0.42)" : "#e8ece6";
      ctx.fillRect(x * cw, y * ch, cw, ch);
      if (showCoverage && (map.coverage_count[y]?.[x] || 0) > 0) {
        const alpha = Math.min(0.62, 0.18 + (map.coverage_count[y][x] || 0) * 0.06);
        ctx.fillStyle = `rgba(55, 132, 206, ${alpha})`;
        ctx.fillRect(x * cw, y * ch, cw, ch);
      }
      if (changed.has(`${x},${y}`)) {
        ctx.fillStyle = "rgba(255, 57, 57, 0.72)";
        ctx.fillRect(x * cw, y * ch, cw, ch);
      }
    }
  }
  ctx.strokeStyle = "rgba(20, 28, 34, 0.08)";
  ctx.lineWidth = 1;
  for (let x = 0; x <= map.width_cells; x += 5) {
    ctx.beginPath();
    ctx.moveTo(x * cw, 0);
    ctx.lineTo(x * cw, height);
    ctx.stroke();
  }
  for (let y = 0; y <= map.height_cells; y += 5) {
    ctx.beginPath();
    ctx.moveTo(0, y * ch);
    ctx.lineTo(width, y * ch);
    ctx.stroke();
  }
}

function drawTrajectories(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  map: SimulationMap,
  trajectories: Record<string, GridPosition[]>,
  selectedUavId?: string,
) {
  Object.entries(trajectories).forEach(([uavId, points], index) => {
    if (points.length < 2) {
      return;
    }
    ctx.strokeStyle = colorFor(index);
    ctx.lineWidth = selectedUavId === uavId ? 4 : 2;
    ctx.globalAlpha = selectedUavId && selectedUavId !== uavId ? 0.22 : 0.72;
    drawPolyline(ctx, width, height, map, points);
    ctx.globalAlpha = 1;
  });
}

function drawPlannedPaths(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  map: SimulationMap,
  commands: Array<ControlCommandSnapshot | ActiveCommandSnapshot>,
  selectedCommandId?: string,
  selectedUavId?: string,
) {
  commands.forEach((command, index) => {
    ctx.strokeStyle = colorFor(index);
    ctx.setLineDash([5, 5]);
    const highlighted = selectedCommandId === command.command_id || selectedUavId === command.uav_id;
    ctx.lineWidth = highlighted ? 4 : 1.5;
    ctx.globalAlpha = selectedCommandId || selectedUavId ? (highlighted ? 0.95 : 0.22) : 0.82;
    drawPolyline(ctx, width, height, map, "remaining_path" in command ? command.remaining_path || [] : command.path || []);
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  });
}

function drawUavs(ctx: CanvasRenderingContext2D, width: number, height: number, map: SimulationMap, uavs: Array<{ id: string; position: GridPosition; status: string }>, selectedUavId?: string) {
  const cw = width / map.width_cells;
  const ch = height / map.height_cells;
  uavs.forEach((uav, index) => {
    const px = (uav.position.x + 0.5) * cw;
    const py = (uav.position.y + 0.5) * ch;
    ctx.fillStyle = statusColor(uav.status, index);
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = selectedUavId === uav.id ? 4 : 2;
    ctx.beginPath();
    ctx.arc(px, py, Math.max(5, Math.min(cw, ch) * 0.72), 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#111820";
    ctx.font = "600 11px ui-monospace, monospace";
    ctx.fillText(uav.id.replace("uav_", ""), px + 7, py - 7);
  });
}

function drawSelection(ctx: CanvasRenderingContext2D, width: number, height: number, map: SimulationMap, start: GridPosition, end: GridPosition) {
  const cw = width / map.width_cells;
  const ch = height / map.height_cells;
  const x = Math.min(start.x, end.x) * cw;
  const y = Math.min(start.y, end.y) * ch;
  const w = (Math.abs(start.x - end.x) + 1) * cw;
  const h = (Math.abs(start.y - end.y) + 1) * ch;
  ctx.fillStyle = "rgba(21, 115, 187, 0.2)";
  ctx.strokeStyle = "#1573bb";
  ctx.lineWidth = 2;
  ctx.fillRect(x, y, w, h);
  ctx.strokeRect(x, y, w, h);
}

function drawPolyline(ctx: CanvasRenderingContext2D, width: number, height: number, map: SimulationMap, points: GridPosition[]) {
  if (points.length < 2) {
    return;
  }
  const cw = width / map.width_cells;
  const ch = height / map.height_cells;
  ctx.beginPath();
  points.forEach((point, index) => {
    const px = (point.x + 0.5) * cw;
    const py = (point.y + 0.5) * ch;
    if (index === 0) {
      ctx.moveTo(px, py);
    } else {
      ctx.lineTo(px, py);
    }
  });
  ctx.stroke();
}

function colorFor(index: number): string {
  return UAV_COLORS[index % UAV_COLORS.length];
}

function statusColor(status: string, index: number): string {
  if (status === "OFFLINE") return "#6f7780";
  if (status === "RETURNING") return "#e25d2f";
  if (status === "CONFIRMING") return "#b84cff";
  if (status === "SEARCHING") return colorFor(index);
  return "#22a06b";
}

function cellInfo(map: SimulationMap, point: GridPosition): string {
  const passable = map.passable[point.y]?.[point.x] ? "passable" : "blocked";
  const coverage = map.coverage_count[point.y]?.[point.x] ?? 0;
  const priority = map.search_priority[point.y]?.[point.x] ?? 1;
  return `${passable} / coverage ${coverage} / priority ${priority}`;
}
